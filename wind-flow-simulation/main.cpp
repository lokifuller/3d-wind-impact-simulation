#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>

#include "shader_configure.h"
#include "camera.h"
#include "ray_triangle.h"

#include "load-model-meshes.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

//--------------------------------------------------
// Problem parameters
//--------------------------------------------------
static const int   Nx = 99 * 5;
static const int   Ny = 99;
static const int   Nz = 99;
static const float Lx = 5.0f;
static const float Ly = 1.0f;
static const float Lz = 1.0f;
static const float Re = 500.0f;
static const float dt = 0.001f;
static const int   MAX_ITER = 10000000;
static const int   PC_ITERS = 150;    // inner pressure‐Poisson sweeps

static const float dx = Lx / Nx;
static const float dy = Ly / Ny;
static const float dz = Lz / Nz;

const unsigned int SCR_WIDTH = 1000;
const unsigned int SCR_HEIGHT = 1000;

// Model Scaling
const float MODEL_SCALE = 2.0f; // gherkin 2.0f

static const float worldLx = MODEL_SCALE * Lx;
static const float worldLy = MODEL_SCALE * Ly;
static const float worldLz = MODEL_SCALE * Lz;

static const float dxw = worldLx / Nx;
static const float dyw = worldLy / Ny;
static const float dzw = worldLz / Nz;
// End Of Model Scaling

float lastFrame = 0.0f;
float deltaTime = 0.0f;

float finalResidual = 1.0f;
const int updateInterval = MAX_ITER / 10;

static std::vector<float> u, v, w;
float computeContinuityResidual();

static const int   ARROW_SUBSAMPLE = 50;
static const float ARROW_VEL_THRESHOLD = 0.001f;

static std::vector<int> solidMask;
static bool carvingDone = false;
static int carveJ = 0, carveK = 0;

static GLuint ssboSolidMask;

//--------------------------------------------------
// SSBO binding points
//--------------------------------------------------
enum {
    BIND_U = 0,
    BIND_V,
    BIND_W,
    BIND_P,
    BIND_USTAR,
    BIND_VSTAR,
    BIND_WSTAR,
    BIND_PPRIME_IN,
    BIND_PPRIME_OUT,
    BIND_SOLID_MASK
};

//--------------------------------------------------
// Helpers to index CFD arrays (for initial upload only)
//--------------------------------------------------
inline int idxU(int i, int j, int k) { return k * Ny * (Nx + 1) + j * (Nx + 1) + i; }
inline int idxV(int i, int j, int k) { return k * (Ny + 1) * Nx + j * Nx + i; }
inline int idxW(int i, int j, int k) { return k * (Ny * Nx) + j * Nx + i; }
inline int idxP(int i, int j, int k) { return k * (Ny * Nx) + j * Nx + i; }

//--------------------------------------------------
// Compile a compute shader from file
//--------------------------------------------------
GLuint compileCompute(const char* path) {
    std::ifstream in(path);
    std::stringstream buf;
    buf << in.rdbuf();
    std::string src = buf.str();
    const char* src_c = src.c_str();

    GLuint sh = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(sh, 1, &src_c, nullptr);
    glCompileShader(sh);
    GLint ok;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(sh, 1024, nullptr, log);
        std::cerr << "Compute shader compile error (" << path << "):\n" << log << "\n";
    }
    GLuint prog = glCreateProgram();
    glAttachShader(prog, sh);
    glLinkProgram(prog);
    glDeleteShader(sh);
    return prog;
}

//--------------------------------------------------
// Create and allocate an SSBO
//--------------------------------------------------
void makeSSBO(GLuint& ssbo, size_t bytes) {
    glGenBuffers(1, &ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, bytes, nullptr, GL_DYNAMIC_COPY);
}


//--------------------------------------------------
// Carve Functions
//--------------------------------------------------
bool loadCarveData() {
    if (solidMask.empty()) return false;
    std::ifstream in("carve_data.txt");
    if (!in) return false;
    for (size_t i = 0; i < solidMask.size(); ++i) {
        int v; in >> v;
        solidMask[i] = char(v);
    }
    std::cout << "Loaded carving data.\n";
    carvingDone = true;

    // Upload to GPU
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboSolidMask);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, solidMask.size() * sizeof(int), solidMask.data());
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    return true;
}

void saveCarveData() {
    std::ofstream out("carve_data.txt");
    for (char c : solidMask) out << int(c) << ' ';
    std::cout << "Saved carving data.\n";
}

void startCarving() {
    std::fill(solidMask.begin(), solidMask.end(), 0);
    carveJ = carveK = 0;
    carvingDone = false;
    std::cout << "Beginning carve pass...\n";
}

void carveOneCell(const std::vector<Triangle>& tris, int j, int k) {
    float cellY = (j + 0.5f) * dyw;
    float cellZ = (k + 0.5f) * dzw;
    // rays start at the cube’s left face x=0 in world‐space
    glm::vec3 origin(0.0f, cellY, cellZ);
    glm::vec3 dir(1.0f, 0.0f, 0.0f);
    std::vector<float> ts;
    ts.reserve(32);
    // collect intersections
    for (auto& T : tris) {
        float minY = std::min({ T.v0.y, T.v1.y, T.v2.y });
        float maxY = std::max({ T.v0.y, T.v1.y, T.v2.y });
        if (cellY < minY || cellY > maxY) continue;
        float tval;
        if (rayTri(origin, dir, T, tval)) ts.push_back(tval);
    }
    if (ts.empty()) return;
    std::sort(ts.begin(), ts.end());
    for (size_t m = 0; m + 1 < ts.size(); m += 2) {
        int i0 = glm::clamp(int(ts[m] / dxw), 0, Nx - 1);
        int i1 = glm::clamp(int(ts[m + 1] / dxw), 0, Nx - 1);
        for (int i = i0; i <= i1; ++i) {
            solidMask[idxP(i, j, k)] = 1;
            u[idxU(i, j, k)] = 0.0f;
            u[idxU(i + 1, j, k)] = 0.0f;
        }
    }
}

void carveChunk(const std::vector<Triangle>& tris, int maxCells) {
    int done = 0;
    while (done++ < maxCells && carveK < Nz) {
        carveOneCell(tris, carveJ, carveK);
        if (++carveJ >= Ny) {
            carveJ = 0; ++carveK;
        }
    }
    if (carveK >= Nz) {
        carvingDone = true;
        std::cout << "Carving complete!\n";
        saveCarveData();

        // count CPU mask entries
        int cpuCount = std::accumulate(solidMask.begin(), solidMask.end(), 0);
        std::cout << "[DEBUG] CPU mask count = " << cpuCount << "\n";

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboSolidMask);
        glBufferSubData(
            GL_SHADER_STORAGE_BUFFER,
            0,
            solidMask.size() * sizeof(int),
            solidMask.data()
        );

        // read back GPU mask (ensure it is same as CPU or there is an error)
        std::vector<int> gpuMask(solidMask.size());
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboSolidMask);
        glGetBufferSubData(
            GL_SHADER_STORAGE_BUFFER,
            0,
            gpuMask.size() * sizeof(int),
            gpuMask.data()
        );
        int gpuCount = std::accumulate(gpuMask.begin(), gpuMask.end(), 0);
        std::cout << "[DEBUG] GPU mask count = " << gpuCount << "\n";
    }
    else {
        std::cout << "Carved row " << carveK << " / " << Nz << "\r";
    }
}


//--------------------------------------------------
// Upload initial fields into SSBOs
//--------------------------------------------------
void initializeFields(GLuint ssboU, GLuint ssboV, GLuint ssboW, GLuint ssboP) {
    std::vector<float> 
        u0((Nx + 1) * Ny * Nz, 0.0f),
        v0(Nx * (Ny + 1) * Nz, 0.0f),
        w0(Nx * Ny * (Nz + 1), 0.0f),
        p0(Nx * Ny * Nz, 0.0f);

    // STARTING MOVEMENT CODE:
    // Inlet Jet: A 20×20 square at the centre of i=0

    /* 
    FIX: Currently sets the inlet to all 1.0f and doesn't follow the same cosine inlet as intended, 
    very small to no effect of outcome of CFD as it exists only as the initial condition, not as the condition
    used for the bulk of calculations (the cosine inlet).
    */
    
    int j0 = (Ny - 20) / 2;
    int k0 = (Nz - 20) / 2;
    for (int k = k0; k < k0 + 20; ++k) {
        for (int j = j0; j < j0 + 20; ++j) {
            u0[idxU(0, j, k)] = 1.0f;
        }
    }

    auto upload = [&](GLuint buf, const std::vector<float>& arr) {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, buf);
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, arr.size() * sizeof(float), arr.data());
        };
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    upload(ssboU, u0);
    upload(ssboV, v0);
    upload(ssboW, w0);
    upload(ssboP, p0);
}

//--------------------------------------------------
// Utility to compute dispatch group count
//--------------------------------------------------
inline int groups(int N) { return (N + 7) / 8; }

//--------------------------------------------------
// Geometry setup and rendering
//--------------------------------------------------
GLuint cubeVAO, cubeVBO;
GLuint pointsVAO, pointsVBO, speedVBO;
GLsizei pointsCount;
GLuint arrowVAO, arrowVBO;
GLuint arrowSpeedVBO;
GLuint maskVAO, maskVBO;
GLuint tipVAO, tipVBO;

void setupGeometry() {
    // Cube edges
    std::vector<glm::vec3> cubeVerts = {
        {0,0,0},{Lx,0,0},{Lx,0,0},{Lx,Ly,0},
        {Lx,Ly,0},{0,Ly,0},{0,Ly,0},{0,0,0},
        {0,0,Lz},{Lx,0,Lz},{Lx,0,Lz},{Lx,Ly,Lz},
        {Lx,Ly,Lz},{0,Ly,Lz},{0,Ly,Lz},{0,0,Lz},
        {0,0,0},{0,0,Lz},{Lx,0,0},{Lx,0,Lz},
        {Lx,Ly,0},{Lx,Ly,Lz},{0,Ly,0},{0,Ly,Lz}
    };
    glGenVertexArrays(1, &cubeVAO);
    glGenBuffers(1, &cubeVBO);
    glBindVertexArray(cubeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, cubeVerts.size() * sizeof(glm::vec3),
        cubeVerts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // Points
    std::vector<glm::vec3> pts;
    pts.reserve(Nx * Ny * Nz);
    for (int k = 0; k < Nz; ++k)
        for (int j = 0; j < Ny; ++j)
            for (int i = 0; i < Nx; ++i)
                pts.push_back({
                    (i + 0.5f) * dx,
                    (j + 0.5f) * dy,
                    (k + 0.5f) * dz
                    });
    pointsCount = (GLsizei)pts.size();

    glGenVertexArrays(1, &pointsVAO);
    glGenBuffers(1, &pointsVBO);
    glGenBuffers(1, &speedVBO);

    glBindVertexArray(pointsVAO);

    // position
    glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
    glBufferData(GL_ARRAY_BUFFER, pts.size() * sizeof(glm::vec3),
        pts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);

    // speed
    glBindBuffer(GL_ARRAY_BUFFER, speedVBO);
    glBufferData(GL_ARRAY_BUFFER, pts.size() * sizeof(float),
        nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    // arrow line geometry
    glGenVertexArrays(1, &arrowVAO);
    glGenBuffers(1, &arrowVBO);
    glGenBuffers(1, &arrowSpeedVBO);
    glBindVertexArray(arrowVAO);
    glBindBuffer(GL_ARRAY_BUFFER, arrowVBO);
    // reserve 2 endpoints per point (base + tip)
    glBufferData(GL_ARRAY_BUFFER,
        2 * pointsCount * sizeof(glm::vec3),
        nullptr,
        GL_DYNAMIC_DRAW);
    // position attribute at location 0
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    // speed attribute at location 1
    glBindBuffer(GL_ARRAY_BUFFER, arrowSpeedVBO);
    glBufferData(GL_ARRAY_BUFFER,
        2 * pointsCount * sizeof(float),
        nullptr,
        GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    // arrow tip
    glGenVertexArrays(1, &tipVAO);
    glGenBuffers(1, &tipVBO);
    glBindVertexArray(tipVAO);
    glBindBuffer(GL_ARRAY_BUFFER, tipVBO);
    glBufferData(GL_ARRAY_BUFFER,
        Nx * Ny * Nz * sizeof(glm::vec3),
        nullptr,
        GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // 3D mesh outline/mask
    glGenVertexArrays(1, &maskVAO);
    glGenBuffers(1, &maskVBO);
    glBindVertexArray(maskVAO);
    glBindBuffer(GL_ARRAY_BUFFER, maskVBO);
    glBufferData(GL_ARRAY_BUFFER,
        Nx * Ny * Nz * sizeof(glm::vec3),
        nullptr,
        GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void renderCube() {
    glBindVertexArray(cubeVAO);
    glDrawArrays(GL_LINES, 0, 24);
    glBindVertexArray(0);
}

void renderPoints() {
    glBindVertexArray(pointsVAO);
    glPointSize(2.0f);
    glDrawArrays(GL_POINTS, 0, pointsCount);
    glBindVertexArray(0);
}

//--------------------------------------------------
// main()
//--------------------------------------------------
int main() {
    
    // GLFW + GLEW
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "3D GPU SIMPLE CFD", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glewInit();
    glEnable(GL_DEPTH_TEST);
    Shader modelShader("vertex_shader.vert", "fragment_shader.frag");
    Model building("gherkinmodel.obj");

    // Model transformation matrix
    float modelHeight = 180.0f;     // Model's original height
    float desiredHeight = 2.0f;     // Desired height multiplier
    float tx = 1.0f;                // move right
    float tz = 1.0f;                // move forward
    float scaleFactor = desiredHeight / modelHeight;

    // Model placement for model rotation, translation, scale
    glm::mat4 modelM = glm::translate(glm::mat4(1.0f),
        glm::vec3(tx, 0.0f, tz))
        * glm::rotate(glm::mat4(1.0f),
            glm::radians(0.0f), // rotation around z-axis gherkin (270.0f) wall (0.0f) walkietalkie (0.0f)
            glm::vec3(1, 0, 0))
        * glm::rotate(glm::mat4(1.0f),
           glm::radians(90.0f), // rotation around y-axis gherkin (0.0f) wall (90.0f) walkietalkie (90.0f)
            glm::vec3(0, 1, 0))
        * glm::scale(glm::mat4(1.0f),
            glm::vec3(scaleFactor));

    // Extract & transform triangles from imported 3D model
    std::vector<Triangle> tris;
    tris.reserve(building.mesh_list.size() * 1000); // rough guess

    // bring mesh verts into world by modelM
    for (auto& mesh : building.mesh_list) {
        for (size_t i = 0; i + 2 < mesh.vert_indices.size(); i += 3) {
            Triangle T;
            // fetch raw positions
            glm::vec3 p0 = mesh.vert_positions[mesh.vert_indices[i + 0]];
            glm::vec3 p1 = mesh.vert_positions[mesh.vert_indices[i + 1]];
            glm::vec3 p2 = mesh.vert_positions[mesh.vert_indices[i + 2]];
            // transform by model matrix
            T.v0 = glm::vec3(modelM * glm::vec4(p0, 1.0f));
            T.v1 = glm::vec3(modelM * glm::vec4(p1, 1.0f));
            T.v2 = glm::vec3(modelM * glm::vec4(p2, 1.0f));
            tris.push_back(T);
        }
    }

    // Camera
    Camera camera(
        glm::vec3(3.0f, 3.0f, 3.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        -135.0f, -35.0f
    );
    glfwSetWindowUserPointer(window, &camera);

    // Create SSBOs
    GLuint ssboU, ssboV, ssboW, ssboP;
    GLuint ssboUStar, ssboVStar, ssboWStar;
    GLuint ssboPPrime[2];
    makeSSBO(ssboU, (Nx + 1) * Ny * Nz * sizeof(float));
    makeSSBO(ssboV, Nx * (Ny + 1) * Nz * sizeof(float));
    makeSSBO(ssboW, Nx * Ny * (Nz + 1) * sizeof(float));
    makeSSBO(ssboP, Nx * Ny * Nz * sizeof(float));
    makeSSBO(ssboUStar, (Nx + 1) * Ny * Nz * sizeof(float));
    makeSSBO(ssboVStar, Nx * (Ny + 1) * Nz * sizeof(float));
    makeSSBO(ssboWStar, Nx * Ny * (Nz + 1) * sizeof(float));
    makeSSBO(ssboPPrime[0], Nx * Ny * Nz * sizeof(float));
    makeSSBO(ssboPPrime[1], Nx * Ny * Nz * sizeof(float));


    solidMask.resize(Nx * Ny * Nz);
    std::fill(solidMask.begin(), solidMask.end(), 0);
    makeSSBO(ssboSolidMask, Nx * Ny * Nz * sizeof(int));
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboSolidMask);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
        solidMask.size() * sizeof(int),
        solidMask.data());
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    {
        std::vector<float> zeroP(Nx * Ny * Nz, 0.0f);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboPPrime[0]);
        glBufferSubData(GL_SHADER_STORAGE_BUFFER,
            0,
            zeroP.size() * sizeof(float),
            zeroP.data());
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboPPrime[1]);
        glBufferSubData(GL_SHADER_STORAGE_BUFFER,
            0,
            zeroP.size() * sizeof(float),
            zeroP.data());
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }

    u.resize((Nx + 1) * Ny * Nz);
    v.resize(Nx * (Ny + 1) * Nz);
    w.resize(Nx * Ny * (Nz + 1));

    // Initialize fields on GPU
    initializeFields(ssboU, ssboV, ssboW, ssboP);

    solidMask.resize(Nx * Ny * Nz);
    if (!loadCarveData()) startCarving();

    // Compile compute shaders
    GLuint progMomX = compileCompute("momentum_x.comp");
    glUseProgram(progMomX);
    int locInletScale = glGetUniformLocation(progMomX, "inletScale");
    GLuint progMomY = compileCompute("momentum_y.comp");
    GLuint progMomZ = compileCompute("momentum_z.comp");

    for (auto prog : { progMomX, progMomY, progMomZ }) {
        glUseProgram(prog);
        glUniform1f(glGetUniformLocation(prog, "alphaU"), 0.7f);
    }

    GLuint progPC = compileCompute("pressure_iter.comp");
    glUseProgram(progPC);
    glUniform1f(glGetUniformLocation(progPC, "alphaP"), 0.3f);
    GLuint progUpdate = compileCompute("update_u.comp");

    GLuint progUpdateV = compileCompute("update_v.comp");
    GLuint progUpdateW = compileCompute("update_w.comp");

    // Set uniforms for each
    auto setCommon = [&](GLuint prog) {
        glUseProgram(prog);
        glUniform1f(glGetUniformLocation(prog, "dx"), dx);
        glUniform1f(glGetUniformLocation(prog, "dy"), dy);
        glUniform1f(glGetUniformLocation(prog, "dz"), dz);
        glUniform1f(glGetUniformLocation(prog, "dt"), dt);
        glUniform1f(glGetUniformLocation(prog, "Re"), Re);
        glUniform1i(glGetUniformLocation(prog, "Nx"), Nx);
        glUniform1i(glGetUniformLocation(prog, "Ny"), Ny);
        glUniform1i(glGetUniformLocation(prog, "Nz"), Nz);
        };
    setCommon(progMomX);
    setCommon(progMomY);
    setCommon(progMomZ);
    setCommon(progPC);
    setCommon(progUpdate);
    setCommon(progUpdateV);
    setCommon(progUpdateW);

    // Bind SSBOs to slots
    auto bindAll = [&]() {
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BIND_U, ssboU);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BIND_V, ssboV);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BIND_W, ssboW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BIND_P, ssboP);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BIND_USTAR, ssboUStar);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BIND_VSTAR, ssboVStar);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BIND_WSTAR, ssboWStar);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BIND_PPRIME_IN, ssboPPrime[0]);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BIND_PPRIME_OUT, ssboPPrime[1]);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BIND_SOLID_MASK, ssboSolidMask);
        };
    bindAll();

    // Setup geometry & load render shaders
    setupGeometry();
    Shader cubeShader("edge.vert", "edge.frag");
    Shader pointShader("velocity.vert", "velocity.frag");
    Shader maskShader("mask.vert", "mask.frag");
    Shader arrowShader("arrow.vert", "arrow.frag");
    Shader tipShader("tip.vert", "tip.frag");

    // Projection
    glm::mat4 projection = glm::perspective(
        glm::radians(75.0f),
        SCR_WIDTH / (float)SCR_HEIGHT,
        0.01f,
        100.0f
    );

    cubeShader.use();  cubeShader.setMat4("projection", projection);
    pointShader.use(); pointShader.setMat4("projection", projection);

    // Mouse callbacks
    glfwSetCursorPosCallback(window,
        [](GLFWwindow* win, double xpos, double ypos) {
            auto cam = static_cast<Camera*>(glfwGetWindowUserPointer(win));
            cam->ProcessMouseMovement((float)xpos, (float)ypos);
        });
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    bool cfdStart = false;
    int  iteration = 0;

    int sw, sh; glfwGetFramebufferSize(window, &sw, &sh);
    glm::mat4 proj = glm::perspective(glm::radians(45.0f), float(sw) / float(sh), 0.1f, 1000.0f);

    // just before main loop, add:
    bool cfdPaused = false;
    bool prevIState = false;

    // in your main loop, very near the top, poll for an *edge* on ‘I’:
    bool iState = (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS);
    if (iState && !prevIState) {
        if (!cfdStart && carvingDone) {
            // first time: start the solver
            cfdStart = true;
            std::cout << "GPU CFD started\n";
        }
        /*
        FIXME: Pause implementation does not currently function as intended.
        */
        else if (cfdStart) {
            // subsequent presses: toggle pause/resume
            cfdPaused = !cfdPaused;
            std::cout << (cfdPaused ? "CFD paused\n" : "CFD resumed\n");
        }
    }
    prevIState = iState;

    // Render loop
    while (!glfwWindowShouldClose(window)) {

        if (!carvingDone) {
            carveChunk(tris, 1024);
        }

        // Time update
        float currentFrame = (float)glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Camera movement
        camera.ProcessKeyboard(window, deltaTime, 0.0f);

        // Start calculations on 'i'
        if (!cfdStart && carvingDone && glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) {
            cfdStart = true;
            std::cout << "GPU CFD started\n";
        }

        // Clear screen
        glClearColor(1, 1, 1, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 5) GPU SIMPLE solver
        if (cfdStart && !cfdPaused && iteration < MAX_ITER) {
            // momentum X with "ramp up"
            float inletScale = iteration < 10000
                ? iteration / 10000.0f
                : 2.0f;
            glUseProgram(progMomX);
            glUniform1f(locInletScale, inletScale);
            glDispatchCompute(groups(Nx + 1), groups(Ny), groups(Nz));
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            // momentum Y
            glUseProgram(progMomY);
            glDispatchCompute(groups(Nx), groups(Ny + 1), groups(Nz));
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            // momentum Z
            glUseProgram(progMomZ);
            glDispatchCompute(groups(Nx), groups(Ny), groups(Nz + 1));
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            // pressure-correction ping‑pong
            int ping = 0;
            for (int pc = 0; pc < PC_ITERS; ++pc) {
                glBindBufferBase(GL_SHADER_STORAGE_BUFFER,
                    BIND_PPRIME_IN, ssboPPrime[ping]);
                glBindBufferBase(GL_SHADER_STORAGE_BUFFER,
                    BIND_PPRIME_OUT, ssboPPrime[ping ^ 1]);
                glUseProgram(progPC);
                glDispatchCompute(groups(Nx), groups(Ny), groups(Nz));
                glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
                ping ^= 1;
            }
            // copy p'->p
            glBindBuffer(GL_COPY_READ_BUFFER, ssboPPrime[ping]);
            glBindBuffer(GL_COPY_WRITE_BUFFER, ssboP);
            glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER,
                0, 0, Nx * Ny * Nz * sizeof(float));
            // update u
            glUseProgram(progUpdate);
            glDispatchCompute(groups(Nx + 1), groups(Ny), groups(Nz));
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

            // update v
            glUseProgram(progUpdateV);
            glDispatchCompute(groups(Nx), groups(Ny + 1), groups(Nz));
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            
            // update w
            glUseProgram(progUpdateW);
            glDispatchCompute(groups(Nx), groups(Ny), groups(Nz + 1));
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

            ++iteration;

            // copy back into CPU u,v,w for vectors and residual calculation
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboU);
            glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, u.size() * sizeof(float), u.data());
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboV);
            glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, v.size() * sizeof(float), v.data());
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboW);
            glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, w.size() * sizeof(float), w.data());
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

            // build & upload mask‐point positions
            std::vector<glm::vec3> maskPts;
            maskPts.reserve(Nx * Ny * Nz);
            int m = 0;
            for (int k = 0; k < Nz; ++k) {
                for (int j = 0; j < Ny; ++j) {
                    for (int i = 0; i < Nx; ++i, ++m) {
                        if (solidMask[m]) {
                            maskPts.push_back({
                              (i + 0.5f) * dx,
                              (j + 0.5f) * dy,
                              (k + 0.5f) * dz
                                });
                        }
                    }
                }
            }
            glBindBuffer(GL_ARRAY_BUFFER, maskVBO);
            glBufferSubData(GL_ARRAY_BUFFER,
                0,
                maskPts.size() * sizeof(glm::vec3),
                maskPts.data());

            // compute & print residual each iteration
            finalResidual = computeContinuityResidual();
            std::cout
                << "Iteration " << iteration
                << " / " << MAX_ITER
                << ", Residual = " << finalResidual
                << "\n";
        }

        // copy SSBO back into CPU arrays (for visualization)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboU);
        glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, u.size() * sizeof(float), u.data());
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboV);
        glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, v.size() * sizeof(float), v.data());
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssboW);
        glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, w.size() * sizeof(float), w.data());
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

        // Build arrow segments
        std::vector<glm::vec3> arrows;
        std::vector<float>     arrowSpeeds;
        arrows.reserve((pointsCount / ARROW_SUBSAMPLE + 1) * 2);
        arrowSpeeds.reserve((pointsCount / ARROW_SUBSAMPLE + 1) * 2);

        int idx = 0;
        for (int k = 0; k < Nz; ++k) {
            for (int j = 0; j < Ny; ++j) {
                for (int i = 0; i < Nx; ++i, ++idx) {
                    // subsample by index
                    if (idx % ARROW_SUBSAMPLE != 0)
                        continue;

                    // compute base position & velocity
                    glm::vec3 base{
                      (i + 0.5f) * dx,
                      (j + 0.5f) * dy,
                      (k + 0.5f) * dz
                    };
                    float ui = 0.5f * (u[idxU(i, j, k)] + u[idxU(i + 1, j, k)]);
                    float vi = 0.5f * (v[idxV(i, j, k)] + v[idxV(i, j + 1, k)]);
                    float wi = 0.5f * (w[idxW(i, j, k)] + w[idxW(i, j, k + 1)]);
                    glm::vec3 vel{ ui, vi, wi };

                    // only draw if speed above threshold
                    float speed = glm::length(vel);
                    if (speed < ARROW_VEL_THRESHOLD)
                        continue;

                    arrows.push_back(base);
                    arrows.push_back(base + vel * 0.6f);  // length scaling

                    // normalized speed per endpoint
                    float normSpeed = glm::clamp(speed, 0.0f, 1.0f);
                    arrowSpeeds.push_back(normSpeed);
                    arrowSpeeds.push_back(normSpeed);
                }
            }
        }

        // upload into arrowVBO
        glBindBuffer(GL_ARRAY_BUFFER, arrowVBO);
        glBufferSubData(GL_ARRAY_BUFFER,
            0,
            arrows.size() * sizeof(glm::vec3),
            arrows.data());

        std::vector<glm::vec3> tipPts;
        tipPts.reserve(arrows.size() / 2);
        for (size_t i = 0; i < arrows.size(); i += 2)
            tipPts.push_back(arrows[i + 1]);

        glBindBuffer(GL_ARRAY_BUFFER, tipVBO);
        glBufferSubData(GL_ARRAY_BUFFER,
            0,
            tipPts.size() * sizeof(glm::vec3),
            tipPts.data());

         // upload into arrowSpeedVBO
        glBindBuffer(GL_ARRAY_BUFFER, arrowSpeedVBO);
        glBufferSubData(GL_ARRAY_BUFFER,
            0,
            arrowSpeeds.size() * sizeof(float),
            arrowSpeeds.data());

        // Build view & model
        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 projection = glm::perspective(
            glm::radians(75.0f),
            SCR_WIDTH / (float)SCR_HEIGHT,
            0.01f, 100.0f
        );

        glm::mat4 cubeModel = glm::scale(glm::mat4(1.0f), glm::vec3(MODEL_SCALE));

        // Build and draw mask points in red. 
        // Comment out for better performance.
        // rebuild maskPts here so it's in scope for drawing
        std::vector<glm::vec3> maskPts;
        maskPts.reserve(Nx* Ny* Nz);
        int m = 0;
        for (int k = 0; k < Nz; ++k) {
            for (int j = 0; j < Ny; ++j) {
                for (int i = 0; i < Nx; ++i, ++m) {
                    if (solidMask[m]) {
                        maskPts.push_back({
                            (i + 0.5f) * dx,
                            (j + 0.5f) * dy,
                            (k + 0.5f) * dz
                            });
                    }
                }
            }
        }
        glBindBuffer(GL_ARRAY_BUFFER, maskVBO);
        glBufferSubData(
            GL_ARRAY_BUFFER,
            0,
            maskPts.size() * sizeof(glm::vec3),
            maskPts.data()
        );

        // draw them
        maskShader.use();
        maskShader.setMat4("model", cubeModel);
        maskShader.setMat4("view", view);
        maskShader.setMat4("projection", projection);
        glBindVertexArray(maskVAO);
        glPointSize(1.0f);
        glDrawArrays(GL_POINTS, 0, GLsizei(maskPts.size()));
        glBindVertexArray(0);
        

        // draw 3D model
        modelShader.use();
        glUniformMatrix4fv(glGetUniformLocation(modelShader.ID, "model"),
            1, GL_FALSE, glm::value_ptr(modelM));
        glUniformMatrix4fv(glGetUniformLocation(modelShader.ID, "view"),
            1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(modelShader.ID, "projection"),
            1, GL_FALSE, glm::value_ptr(projection));
        for (auto& mesh : building.mesh_list) {
            glBindVertexArray(mesh.VAO);
            glDrawElements(GL_TRIANGLES,
                GLsizei(mesh.vert_indices.size()),
                GL_UNSIGNED_INT,
                nullptr);
        }
        glBindVertexArray(0);

        // Render cube
        cubeShader.use();
        cubeShader.setMat4("model", cubeModel);
        cubeShader.setMat4("view", view);
        cubeShader.setMat4("projection", projection);
        renderCube();

        // Render velocity lines
        arrowShader.use();
        arrowShader.setMat4("model", cubeModel);
        arrowShader.setMat4("view", view);
        arrowShader.setMat4("projection", projection);
        glBindVertexArray(arrowVAO);
        glLineWidth(3.0f);
        glDrawArrays(GL_LINES, 0, GLsizei(arrows.size()));
        glBindVertexArray(0);
        
        // Draw pink tips
        tipShader.use();
        tipShader.setMat4("model", cubeModel);
        tipShader.setMat4("view", view);
        tipShader.setMat4("projection", projection);
        glBindVertexArray(tipVAO);
        glDrawArrays(GL_POINTS, 0, GLsizei(tipPts.size()));
        glBindVertexArray(0);

        // Swap & poll
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // cleanup
    glfwTerminate();
    return 0;
}

float computeContinuityResidual() {
    double sumSq = 0.0;
    for (int k = 0; k < Nz; ++k) {
        for (int j = 0; j < Ny; ++j) {
            for (int i = 0; i < Nx; ++i) {
                float du = (u[idxU(i + 1, j, k)] - u[idxU(i, j, k)]) / dx;
                float dv = (v[idxV(i, j + 1, k)] - v[idxV(i, j, k)]) / dy;
                float dw = (w[idxW(i, j, k + 1)] - w[idxW(i, j, k)]) / dz;
                float div = du + dv + dw;
                sumSq += double(div) * double(div);
            }
        }
    }
    return float(std::sqrt(sumSq / (Nx * Ny * Nz)));
}
