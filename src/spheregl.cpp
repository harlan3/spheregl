//
// Controls:
//   - Mouse wheel: zoom (or +/- keys if wheel not supported)
//   - Left drag: rotate the sphere (yaw/pitch)
//   - L: toggle leaf-only labels
//   - F: toggle fullscreen
//   - R: toggle rotation animation (spins the sphere)
//   - [ / ]: rotation speed down/up
//   - C: toggle curved links vs straight links
//   - ESC: quit
//

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

#include "tinyxml2.h"

#include <GL/glut.h>
#include <GL/glu.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------- Config ----------------------------

static bool  LABEL_LEAVES_ONLY  = false;    // press 'L' to toggle leaf-only labels

static float SPHERE_RADIUS      = 260.0f;   // world units
static float SPHERE_THETA_MAX   = 0.55f * float(M_PI);

static bool  LINKS_CURVED       = true;     // press 'C' to toggle
static int   LINK_SAMPLES       = 28;

static void* LABEL_STROKE_FONT  = GLUT_STROKE_ROMAN;

// Stroke units are roughly ~100 tall for GLUT_STROKE_ROMAN.
// We'll compute a world-space scale per label so they remain readable.
static float LABEL_BASE_SCALE_PER_DIST = 0.00001f;  // distance -> scale factor
static float LABEL_SCALE_MIN = 0.02f;
static float LABEL_SCALE_MAX = 0.02f;

// How far "above" the sphere to draw glyphs to avoid z-fighting, while still depth-tested.
static float LABEL_SURFACE_EPSILON = 0.85f;

// How far from the node endpoint (along the label direction on the sphere surface)
// to start the label. This is a geodesic distance measured in WORLD UNITS along the sphere.
static float LABEL_START_OFFSET_WORLD = 3.0f;

// Extra spacing between glyphs as a fraction of each glyph's nominal advance.
static float LABEL_GLYPH_SPACING = 0.06f;

// Slight fade so labels feel blended into the sphere surface.
static float LABEL_ALPHA = 0.60f;

static float ENDPOINT_RADIUS    = 1.0f;

static float BASE_CAM_DIST      = 900.0f;

static bool  DRAW_WIREFRAME_SPHERE = true;

// ---------------------------- Data Model ----------------------------

struct Node {
    std::string id;
    std::string text;
    Node* parent = nullptr;
    std::vector<std::unique_ptr<Node>> children;

    int depth = 0;
    int leafCount = 0;

    float angle = 0.0f; // azimuth
    float theta = 0.0f; // polar

    float x = 0.0f, y = 0.0f, z = 0.0f; // world position on sphere

    bool visible = false; // camera-visible hemisphere this frame
};

static int g_autoId = 1;
static std::unique_ptr<Node> g_root;

// ---------------------------- Window / Camera / Interaction ----------------------------

static int   g_winW = 1000;
static int   g_winH = 900;

static float g_zoom = 1.0f;
static float g_yawDeg = 0.0f;
static float g_pitchDeg = 18.0f;

static bool  g_dragging = false;
static int   g_lastMouseX = 0, g_lastMouseY = 0;

static bool g_fullscreen = false;
static int  g_winX = 100, g_winY = 100;
static int  g_winW_prev = 1000, g_winH_prev = 900;

static bool  g_rotateAnim = false;
static float g_rotDeg = 0.0f;
static float g_rotDegPerSec = 15.0f;
static int   g_lastTimeMs = 0;

// ---------------------------- Helpers ----------------------------

static float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

static float dot3(float ax, float ay, float az, float bx, float by, float bz) {
    return ax*bx + ay*by + az*bz;
}

static void cross3(float ax, float ay, float az,
                   float bx, float by, float bz,
                   float& ox, float& oy, float& oz)
{
    ox = ay*bz - az*by;
    oy = az*bx - ax*bz;
    oz = ax*by - ay*bx;
}

static float len3(float x, float y, float z) {
    return std::sqrt(x*x + y*y + z*z);
}

static void normalize3(float& x, float& y, float& z) {
    float l = len3(x,y,z);
    if (l > 1e-9f) { x/=l; y/=l; z/=l; }
}

static void multMatPoint3(const GLdouble* m, double x, double y, double z,
                          double& ox, double& oy, double& oz)
{
    // column-major
    ox = m[0]*x + m[4]*y + m[8]*z  + m[12];
    oy = m[1]*x + m[5]*y + m[9]*z  + m[13];
    oz = m[2]*x + m[6]*y + m[10]*z + m[14];
}

// Visible hemisphere test in EYE space (camera at origin).
static bool isPointOnVisibleHemisphere_EyeSpace(double px, double py, double pz,
                                                const double centerEye[3])
{
    // Outward normal at P is (P - C)
    double nx = px - centerEye[0];
    double ny = py - centerEye[1];
    double nz = pz - centerEye[2];

    double nlen = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (nlen < 1e-9) return false;
    nx /= nlen; ny /= nlen; nz /= nlen;

    // Vector from point to camera is -P
    double vx = -px, vy = -py, vz = -pz;
    double vlen = std::sqrt(vx*vx + vy*vy + vz*vz);
    if (vlen < 1e-9) return false;
    vx /= vlen; vy /= vlen; vz /= vlen;

    return (nx*vx + ny*vy + nz*vz) > 0.0;
}

// Move on a unit sphere by arc length "a" (radians) along a tangent direction d (unit, perpendicular to p).
// Great-circle step:
//   p' = cos(a)*p + sin(a)*d
// and parallel-transport the tangent direction:
//   d' = cos(a)*d - sin(a)*p
static void greatCircleStep(float a,
                            float& px, float& py, float& pz,
                            float& dx, float& dy, float& dz)
{
    float ca = std::cos(a);
    float sa = std::sin(a);

    float npx = ca*px + sa*dx;
    float npy = ca*py + sa*dy;
    float npz = ca*pz + sa*dz;

    float ndx = ca*dx - sa*px;
    float ndy = ca*dy - sa*py;
    float ndz = ca*dz - sa*pz;

    px = npx; py = npy; pz = npz;
    dx = ndx; dy = ndy; dz = ndz;

    // numeric cleanup
    normalize3(px,py,pz);
    // keep d tangent
    float proj = px*dx + py*dy + pz*dz;
    dx -= proj*px; dy -= proj*py; dz -= proj*pz;
    normalize3(dx,dy,dz);
}

// ---------------------------- Stroke Text ----------------------------

static float strokeCharAdvance(void* font, unsigned char c) {
    return float(glutStrokeWidth(font, c));
}

// ---------------------------- XML Parsing (FreeMind) ----------------------------

static std::string getAttr(tinyxml2::XMLElement* el, const char* name) {
    const char* v = el->Attribute(name);
    return v ? std::string(v) : std::string();
}

static std::unique_ptr<Node> parseNode(tinyxml2::XMLElement* xmlNode, Node* parent) {
    auto n = std::make_unique<Node>();
    n->parent = parent;

    n->text = getAttr(xmlNode, "TEXT");
    n->id   = getAttr(xmlNode, "ID");

    if (n->id.empty()) n->id = "auto_" + std::to_string(g_autoId++);
    if (n->text.empty()) n->text = n->id;

    for (tinyxml2::XMLElement* c = xmlNode->FirstChildElement("node"); c; c = c->NextSiblingElement("node")) {
        auto child = parseNode(c, n.get());
        n->children.insert(n->children.begin(), std::move(child));
    }
    return n;
}

static std::unique_ptr<Node> loadFreeMind(const char* path) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path) != tinyxml2::XML_SUCCESS) {
        std::fprintf(stderr, "Failed to load %s\n", path);
        return nullptr;
    }
    auto* mapEl = doc.FirstChildElement("map");
    if (!mapEl) { std::fprintf(stderr, "No <map> element.\n"); return nullptr; }

    auto* rootEl = mapEl->FirstChildElement("node");
    if (!rootEl) { std::fprintf(stderr, "No root <node> element.\n"); return nullptr; }

    return parseNode(rootEl, nullptr);
}

// ---------------------------- Layout ----------------------------

static int computeDepthAndLeaves(Node* n, int depth) {
    n->depth = depth;
    if (n->children.empty()) { n->leafCount = 1; return 1; }

    int sum = 0;
    for (auto& ch : n->children) sum += computeDepthAndLeaves(ch.get(), depth + 1);
    n->leafCount = std::max(1, sum);
    return n->leafCount;
}

static void assignAngles(Node* n, float a0, float a1) {
    n->angle = 0.5f * (a0 + a1);
    if (n->children.empty()) return;

    float span = (a1 - a0);
    float cur = a0;

    int totalLeaves = 0;
    for (auto& ch : n->children) totalLeaves += ch->leafCount;
    totalLeaves = std::max(1, totalLeaves);

    for (auto& ch : n->children) {
        float frac = float(ch->leafCount) / float(totalLeaves);
        float next = cur + span * frac;
        assignAngles(ch.get(), cur, next);
        cur = next;
    }
}

static int g_maxDepth = 1;

static void computeMaxDepth(const Node* n) {
    g_maxDepth = std::max(g_maxDepth, n->depth);
    for (const auto& ch : n->children) computeMaxDepth(ch.get());
}

static void projectToSphere(Node* n) {
    float depthFrac = (g_maxDepth > 0) ? (float(n->depth) / float(g_maxDepth)) : 0.0f;
    n->theta = depthFrac * SPHERE_THETA_MAX;
    float phi = n->angle;

    float st = std::sin(n->theta);
    float ct = std::cos(n->theta);

    n->x = SPHERE_RADIUS * st * std::cos(phi);
    n->y = SPHERE_RADIUS * ct;
    n->z = SPHERE_RADIUS * st * std::sin(phi);

    for (auto& ch : n->children) projectToSphere(ch.get());
}

static void computeLayout() {
    computeDepthAndLeaves(g_root.get(), 0);
    g_maxDepth = 1;
    computeMaxDepth(g_root.get());
    assignAngles(g_root.get(), 0.0f, 2.0f * float(M_PI));
    projectToSphere(g_root.get());
}

// ---------------------------- Visibility Marking ----------------------------

static void markVisibilityRecursive(Node* n, const GLdouble* model, const double centerEye[3]) {
    double pex, pey, pez;
    multMatPoint3(model, (double)n->x, (double)n->y, (double)n->z, pex, pey, pez);
    n->visible = isPointOnVisibleHemisphere_EyeSpace(pex, pey, pez, centerEye);
    for (auto& ch : n->children) markVisibilityRecursive(ch.get(), model, centerEye);
}

// ---------------------------- Link Drawing (visibility-aware) ----------------------------

static void slerpDir(float ax, float ay, float az,
                     float bx, float by, float bz,
                     float t,
                     float& ox, float& oy, float& oz)
{
    normalize3(ax, ay, az);
    normalize3(bx, by, bz);

    float d = clampf(dot3(ax, ay, az, bx, by, bz), -1.0f, 1.0f);
    float omega = std::acos(d);
    if (omega < 1e-6f) { ox = ax; oy = ay; oz = az; return; }

    float so = std::sin(omega);
    float s0 = std::sin((1.0f - t) * omega) / so;
    float s1 = std::sin(t * omega) / so;

    ox = s0*ax + s1*bx;
    oy = s0*ay + s1*by;
    oz = s0*az + s1*bz;
    normalize3(ox, oy, oz);
}

static void drawLinkStraight(const Node* parent, const Node* child) {
    glBegin(GL_LINES);
    glVertex3f(parent->x, parent->y, parent->z);
    glVertex3f(child->x,  child->y,  child->z);
    glEnd();
}

static void drawLinkGreatCircle(const Node* parent, const Node* child) {
    float ax = parent->x, ay = parent->y, az = parent->z;
    float bx = child->x,  by = child->y,  bz = child->z;

    float adx = ax / SPHERE_RADIUS, ady = ay / SPHERE_RADIUS, adz = az / SPHERE_RADIUS;
    float bdx = bx / SPHERE_RADIUS, bdy = by / SPHERE_RADIUS, bdz = bz / SPHERE_RADIUS;

    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= LINK_SAMPLES; ++i) {
        float t = float(i) / float(LINK_SAMPLES);
        float ux, uy, uz;
        slerpDir(adx, ady, adz, bdx, bdy, bdz, t, ux, uy, uz);
        glVertex3f(ux * SPHERE_RADIUS, uy * SPHERE_RADIUS, uz * SPHERE_RADIUS);
    }
    glEnd();
}

static void drawEdgesRecursiveVisible(const Node* n) {
    for (const auto& ch : n->children) {
        const Node* c = ch.get();
        if (n->visible && c->visible) {
            glColor4f(0.1f, 0.1f, 0.1f, 0.55f);
            glLineWidth(1.0f);
            if (LINKS_CURVED) drawLinkGreatCircle(n, c);
            else              drawLinkStraight(n, c);
        }
        drawEdgesRecursiveVisible(c);
    }
}

static void drawSpheresRecursiveVisible(const Node* n) {
    if (n->visible) {
        glColor4f(0.30f, 0.30f, 0.30f, 0.95f);
        glPushMatrix();
        glTranslatef(n->x, n->y, n->z);
        glutSolidSphere(ENDPOINT_RADIUS, 10, 10);
        glPopMatrix();
    }
    for (const auto& ch : n->children) drawSpheresRecursiveVisible(ch.get());
}

// ---------------------------- Label Drawing (WRAPPED onto sphere surface) ----------------------------

static void drawOneLabelWrappedOnSphere(const Node* n, const GLdouble* model)
{
    // Unit normal at node position (sphere center at origin)
    float px = n->x / SPHERE_RADIUS;
    float py = n->y / SPHERE_RADIUS;
    float pz = n->z / SPHERE_RADIUS;
    normalize3(px, py, pz);

    // Choose local "down" direction as projection of world -Y onto the tangent plane.
    float dx = 0.0f, dy = -1.0f, dz = 0.0f;
    float proj = dx*px + dy*py + dz*pz;
    dx -= proj*px; dy -= proj*py; dz -= proj*pz;
    float dlen = len3(dx,dy,dz);
    if (dlen < 1e-6f) {
        // near poles: project world +Z instead
        dx = 0.0f; dy = 0.0f; dz = 1.0f;
        proj = dx*px + dy*py + dz*pz;
        dx -= proj*px; dy -= proj*py; dz -= proj*pz;
        dlen = len3(dx,dy,dz);
    }
    if (dlen < 1e-6f) {
        dx = 1.0f; dy = 0.0f; dz = 0.0f;
    } else {
        dx/=dlen; dy/=dlen; dz/=dlen;
    }

    // Compute eye-space distance to scale the label (stable with zoom/camera distance).
    double ex, ey, ez;
    multMatPoint3(model, (double)n->x, (double)n->y, (double)n->z, ex, ey, ez);
    float dist = float(std::sqrt(ex*ex + ey*ey + ez*ez));

    float scale = clampf(dist * LABEL_BASE_SCALE_PER_DIST, LABEL_SCALE_MIN, LABEL_SCALE_MAX);

    // Start marching from node endpoint, but offset a bit along label direction (configurable).
    // Convert world distance along surface to arc radians.
    float startOffsetArc = (LABEL_START_OFFSET_WORLD / SPHERE_RADIUS);

    float ptx = px, pty = py, ptz = pz;
    float dtx = dx, dty = dy, dtz = dz;

    if (std::fabs(startOffsetArc) > 1e-9f) {
        greatCircleStep(startOffsetArc, ptx, pty, ptz, dtx, dty, dtz);
    }

    glColor4f(0.10f, 0.10f, 0.10f, LABEL_ALPHA);

    for (unsigned char c : n->text) {
        float adv = strokeCharAdvance(LABEL_STROKE_FONT, c);
        float glyphWorldLen = adv * scale;
        float glyphArc = glyphWorldLen / SPHERE_RADIUS;

        // Place glyph at mid of its advance for nicer spacing
        float midStep = 0.5f * glyphArc;
        float pcharx = ptx, pchary = pty, pcharz = ptz;
        float dcharx = dtx, dchary = dty, dcharz = dtz;
        greatCircleStep(midStep, pcharx, pchary, pcharz, dcharx, dchary, dcharz);

        // Local normal = pchar (unit)
        float nx = pcharx, ny = pchary, nz = pcharz;
        normalize3(nx, ny, nz);

        // Baseline direction for GLUT stroke (+X) is our "down" tangent => top->bottom.
        float bx = dcharx, by = dchary, bz = dcharz;
        normalize3(bx, by, bz);

        // Build a right-handed frame: X = baseline (down), Z = normal, Y = Z x X
        float yx, yy, yz;
        cross3(nx, ny, nz, bx, by, bz, yx, yy, yz);
        normalize3(yx, yy, yz);

        // Re-orthonormalize X = Y x Z (helps drift)
        cross3(yx, yy, yz, nx, ny, nz, bx, by, bz);
        normalize3(bx, by, bz);

        // World position (slightly above surface to avoid z-fighting but still depth-tested)
        float wx = (pcharx * SPHERE_RADIUS) + nx * LABEL_SURFACE_EPSILON;
        float wy = (pchary * SPHERE_RADIUS) + ny * LABEL_SURFACE_EPSILON;
        float wz = (pcharz * SPHERE_RADIUS) + nz * LABEL_SURFACE_EPSILON;

        GLfloat M[16] = {
            bx,    by,    bz,    0.0f,
            yx,    yy,    yz,    0.0f,
            nx,    ny,    nz,    0.0f,
            0.0f,  0.0f,  0.0f,  1.0f
        };

        glPushMatrix();
        glTranslatef(wx, wy, wz);
        glMultMatrixf(M);
        glScalef(scale, scale, 1.0f);

        glutStrokeCharacter(LABEL_STROKE_FONT, c);

        glPopMatrix();

        // Advance to next glyph start: step by glyphArc plus spacing
        float stepArc = glyphArc * (1.0f + LABEL_GLYPH_SPACING);
        greatCircleStep(stepArc, ptx, pty, ptz, dtx, dty, dtz);
    }
}

static void drawLabelsRecursive_WrappedOnSphere(const Node* n, const GLdouble* model)
{
    if (n->visible) {
        bool isLeaf = n->children.empty();
        if (!LABEL_LEAVES_ONLY || isLeaf || (n == g_root.get())) {
            drawOneLabelWrappedOnSphere(n, model);
        }
    }
    for (const auto& ch : n->children)
        drawLabelsRecursive_WrappedOnSphere(ch.get(), model);
}

// ---------------------------- Rendering ----------------------------

static void setup3D() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = (g_winH != 0) ? float(g_winW) / float(g_winH) : 1.0f;
    gluPerspective(45.0, aspect, 0.1, 8000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    float camDist = (BASE_CAM_DIST / g_zoom);
    glTranslatef(0.0f, 0.0f, -camDist);

    glRotatef(g_pitchDeg, 1.0f, 0.0f, 0.0f);
    glRotatef(g_yawDeg,   0.0f, 1.0f, 0.0f);

    glRotatef(g_rotDeg, 0.0f, 1.0f, 0.0f);
}

static void display() {
    glClearColor(1,1,1,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    setup3D();

    GLdouble model[16], proj[16];
    GLint viewport[4];
    glGetDoublev(GL_MODELVIEW_MATRIX, model);
    glGetDoublev(GL_PROJECTION_MATRIX, proj);
    glGetIntegerv(GL_VIEWPORT, viewport);

    double cex, cey, cez;
    multMatPoint3(model, 0.0, 0.0, 0.0, cex, cey, cez);
    double centerEye[3] = { cex, cey, cez };

    markVisibilityRecursive(g_root.get(), model, centerEye);

    if (DRAW_WIREFRAME_SPHERE) {
        glColor4f(0.55f, 0.55f, 0.55f, 0.25f);
        glPushMatrix();
            glTranslatef(0.0, 0.0, 0.0);
            glRotatef(90.0, 1.0, 0.0, 0.0);
            glutWireSphere(SPHERE_RADIUS, 18, 14);
        glPopMatrix();
    }

    drawEdgesRecursiveVisible(g_root.get());
    drawSpheresRecursiveVisible(g_root.get());

    drawLabelsRecursive_WrappedOnSphere(g_root.get(), model);

    glutSwapBuffers();
}

// ---------------------------- Animation ----------------------------

static void idle() {
    if (!g_rotateAnim) return;

    int now = glutGet(GLUT_ELAPSED_TIME);
    if (g_lastTimeMs == 0) g_lastTimeMs = now;

    int dtMs = now - g_lastTimeMs;
    g_lastTimeMs = now;

    float dt = float(dtMs) / 1000.0f;
    g_rotDeg += g_rotDegPerSec * dt;

    if (g_rotDeg >= 360.0f) g_rotDeg -= 360.0f;
    if (g_rotDeg < 0.0f)    g_rotDeg += 360.0f;

    glutPostRedisplay();
}

// ---------------------------- Interaction ----------------------------

static void reshape(int w, int h) {
    g_winW = std::max(1, w);
    g_winH = std::max(1, h);
    glViewport(0, 0, g_winW, g_winH);
    glutPostRedisplay();
}

static void keyboard(unsigned char key, int, int) {
    if (key == 27) std::exit(0); // ESC

    if (key == '+' || key == '=') g_zoom = std::min(20.0f, g_zoom * 1.1f);
    if (key == '-' || key == '_') g_zoom = std::max(0.1f,  g_zoom * 0.9f);

    if (key == 'l' || key == 'L') LABEL_LEAVES_ONLY = !LABEL_LEAVES_ONLY;

    if (key == 'c' || key == 'C') LINKS_CURVED = !LINKS_CURVED;

    if (key == 'f' || key == 'F') {
        if (!g_fullscreen) {
            g_fullscreen = true;
            g_winW_prev = g_winW;
            g_winH_prev = g_winH;
            g_winX = glutGet(GLUT_WINDOW_X);
            g_winY = glutGet(GLUT_WINDOW_Y);
            glutFullScreen();
        } else {
            g_fullscreen = false;
            glutReshapeWindow(g_winW_prev, g_winH_prev);
            glutPositionWindow(g_winX, g_winY);
        }
    }

    if (key == 'r' || key == 'R') {
        g_rotateAnim = !g_rotateAnim;
        g_lastTimeMs = 0;
    }

    if (key == '[') g_rotDegPerSec = std::max(0.0f, g_rotDegPerSec - 5.0f);
    if (key == ']') g_rotDegPerSec = std::min(360.0f, g_rotDegPerSec + 5.0f);

    glutPostRedisplay();
}

static void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            g_dragging = true;
            g_lastMouseX = x;
            g_lastMouseY = y;
        } else {
            g_dragging = false;
        }
    }

    // Mouse wheel (FreeGLUT uses buttons 3/4)
    if (state == GLUT_DOWN) {
        if (button == 3) {
            g_zoom = std::min(20.0f, g_zoom * 1.1f);
            glutPostRedisplay();
        } else if (button == 4) {
            g_zoom = std::max(0.1f,  g_zoom * 0.9f);
            glutPostRedisplay();
        }
    }
}

static void motion(int x, int y) {
    if (!g_dragging) return;

    int dx = x - g_lastMouseX;
    int dy = y - g_lastMouseY;
    g_lastMouseX = x;
    g_lastMouseY = y;

    g_yawDeg   += float(dx) * 0.35f;
    g_pitchDeg += float(dy) * 0.35f;
    g_pitchDeg = clampf(g_pitchDeg, -89.0f, 89.0f);

    glutPostRedisplay();
}

// ---------------------------- Main ----------------------------

int main(int argc, char** argv) {
    const char* path = (argc >= 2) ? argv[1] : "example.mm";

    g_root = loadFreeMind(path);
    if (!g_root) return 1;

    computeLayout();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(g_winW, g_winH);
    glutInitWindowPosition(g_winX, g_winY);
    glutCreateWindow("Sphere Graph");

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutIdleFunc(idle);

    glutMainLoop();
    return 0;
}