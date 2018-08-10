// system includes
#include <stdio.h>

// system includes
#include <smpl/debug/visualize.h>

class TextVisualizer : public smpl::visual::VisualizerBase
{
public:

    void visualize(smpl::visual::Level level, const smpl::visual::Marker& marker)
    {
        switch (level) {
        case smpl::visual::Level::Debug:    printf("[DEBUG] "); break;
        case smpl::visual::Level::Info:     printf("[INFO]  "); break;
        case smpl::visual::Level::Warn:     printf("[WARN]  "); break;
        case smpl::visual::Level::Error:    printf("[ERROR] "); break;
        case smpl::visual::Level::Fatal:    printf("[FATAL] "); break;
        default:                            assert(0); break;
        }
        printf("[%s] ", marker.ns.c_str());
        switch (type(marker.shape)) {
            case smpl::visual::ShapeType::SHAPE_ARROW:
            printArrow(marker);
            break;
        case smpl::visual::ShapeType::SHAPE_CUBE:
            printCube(marker);
            break;
        case smpl::visual::ShapeType::SHAPE_CUBE_LIST:
            printCubeList(marker);
            break;
        case smpl::visual::ShapeType::SHAPE_CYLINDER:
            printCylinder(marker);
            break;
        case smpl::visual::ShapeType::SHAPE_LINE_LIST:
            printLineList(marker);
            break;
        case smpl::visual::ShapeType::SHAPE_LINE_STRIP:
            printLineStrip(marker);
            break;
        case smpl::visual::ShapeType::SHAPE_MESH_RESOURCE:
            printMeshResource(marker);
            break;
        case smpl::visual::ShapeType::SHAPE_POINT_LIST:
            printPoints(marker);
            break;
        case smpl::visual::ShapeType::SHAPE_SPHERE:
            printSphere(marker);
            break;
        case smpl::visual::ShapeType::SHAPE_SPHERE_LIST:
            printSphereList(marker);
            break;
        case smpl::visual::ShapeType::SHAPE_BILLBOARD_TEXT:
            printTextViewFacing(marker);
            break;
        case smpl::visual::ShapeType::SHAPE_TRIANGLE_LIST:
            printTriangleList(marker);
            break;
        default:
            assert(0);
            break;
        }
        printf("\n");
    }

private:

    void printArrow(const smpl::visual::Marker& m)
    {
        printf("arrow");
    }

    void printCube(const smpl::visual::Marker& m)
    {
        printf("cube");
    }

    void printCubeList(const smpl::visual::Marker& m)
    {
        printf("cube list");
    }

    void printCylinder(const smpl::visual::Marker& m)
    {
        printf("cylinder");
    }

    void printLineList(const smpl::visual::Marker& m)
    {
        printf("line list");
    }

    void printLineStrip(const smpl::visual::Marker& m)
    {
        printf("line strip");
    }

    void printMeshResource(const smpl::visual::Marker& m)
    {
        printf("mesh resource");
    }

    void printPoints(const smpl::visual::Marker& m)
    {
        printf("points");
    }

    void printSphere(const smpl::visual::Marker& m)
    {
        printf("sphere");
    }

    void printSphereList(const smpl::visual::Marker& m)
    {
        printf("sphere list");
    }

    void printTextViewFacing(const smpl::visual::Marker& m)
    {
        printf("text view facing");
    }

    void printTriangleList(const smpl::visual::Marker& m)
    {
        printf("triangle list");
    }
};

auto CreateTestMarker() -> smpl::visual::Marker
{
    auto m = smpl::visual::Marker{ };
    m.shape = smpl::visual::Sphere{ };
    return m;
}

auto CreateTestMarkers() -> std::vector<smpl::visual::Marker>
{
    std::vector<smpl::visual::Marker> ma;
    smpl::visual::Marker m;
    m.shape = smpl::visual::Arrow{ };
    ma.push_back(m);
    m.shape = smpl::visual::Cube{ };
    ma.push_back(m);
    m.shape = smpl::visual::Sphere{ };
    ma.push_back(m);
    return ma;
}

void TestBareMacros()
{
    auto ma = CreateTestMarker();
    SV_SHOW_DEBUG(ma);
    SV_SHOW_INFO(ma);
    SV_SHOW_WARN(ma);
    SV_SHOW_ERROR(ma);
    SV_SHOW_FATAL(ma);
}

void TestCondMacros()
{
    auto ma = CreateTestMarker();
    SV_SHOW_DEBUG_COND(true, ma);
    SV_SHOW_INFO_COND(true, ma);
    SV_SHOW_WARN_COND(true, ma);
    SV_SHOW_ERROR_COND(true, ma);
    SV_SHOW_FATAL_COND(true, ma);

    SV_SHOW_DEBUG_COND(false, ma);
    SV_SHOW_INFO_COND(false, ma);
    SV_SHOW_WARN_COND(false, ma);
    SV_SHOW_ERROR_COND(false, ma);
    SV_SHOW_FATAL_COND(false, ma);
}

void TestOnceMacros()
{
    auto ma = CreateTestMarker();
    for (int i = 0; i < 10; ++i) {
        SV_SHOW_DEBUG_ONCE(ma);
        SV_SHOW_INFO_ONCE(ma);
        SV_SHOW_WARN_ONCE(ma);
        SV_SHOW_ERROR_ONCE(ma);
        SV_SHOW_FATAL_ONCE(ma);
    }
}

void TestThrottleMacros()
{
    auto ma = CreateTestMarker();

    auto start = smpl::clock::now();
    while (smpl::clock::now() < start + std::chrono::milliseconds(500)) {
        SV_SHOW_DEBUG_THROTTLE(10.0, ma);
        SV_SHOW_INFO_THROTTLE(10.0, ma);
        SV_SHOW_WARN_THROTTLE(10.0, ma);
        SV_SHOW_ERROR_THROTTLE(10.0, ma);
        SV_SHOW_FATAL_THROTTLE(10.0, ma);
    }
}

void TestNamedMacros()
{
    auto ma = CreateTestMarker();

    namespace sviz = smpl::visual;

    sviz::set_visualization_level(SV_NAME_PREFIX ".debug", sviz::Level::Debug);
    sviz::set_visualization_level(SV_NAME_PREFIX ".info", sviz::Level::Info);
    sviz::set_visualization_level(SV_NAME_PREFIX ".warn", sviz::Level::Warn);
    sviz::set_visualization_level(SV_NAME_PREFIX ".error", sviz::Level::Error);
    sviz::set_visualization_level(SV_NAME_PREFIX ".fatal", sviz::Level::Fatal);

    ma.ns = "debug";
    SV_SHOW_DEBUG_NAMED("debug", ma);
    SV_SHOW_INFO_NAMED("debug", ma);
    SV_SHOW_WARN_NAMED("debug", ma);
    SV_SHOW_ERROR_NAMED("debug", ma);
    SV_SHOW_FATAL_NAMED("debug", ma);

    ma.ns = "info";
    SV_SHOW_DEBUG_NAMED("info", ma);
    SV_SHOW_INFO_NAMED("info", ma);
    SV_SHOW_WARN_NAMED("info", ma);
    SV_SHOW_ERROR_NAMED("info", ma);
    SV_SHOW_FATAL_NAMED("info", ma);

    ma.ns = "warn";
    SV_SHOW_DEBUG_NAMED("warn", ma);
    SV_SHOW_INFO_NAMED("warn", ma);
    SV_SHOW_WARN_NAMED("warn", ma);
    SV_SHOW_ERROR_NAMED("warn", ma);
    SV_SHOW_FATAL_NAMED("warn", ma);

    ma.ns = "error";
    SV_SHOW_DEBUG_NAMED("error", ma);
    SV_SHOW_INFO_NAMED("error", ma);
    SV_SHOW_WARN_NAMED("error", ma);
    SV_SHOW_ERROR_NAMED("error", ma);
    SV_SHOW_FATAL_NAMED("error", ma);

    ma.ns = "fatal";
    SV_SHOW_DEBUG_NAMED("fatal", ma);
    SV_SHOW_INFO_NAMED("fatal", ma);
    SV_SHOW_WARN_NAMED("fatal", ma);
    SV_SHOW_ERROR_NAMED("fatal", ma);
    SV_SHOW_FATAL_NAMED("fatal", ma);
}

void TestNamedCondMacros()
{
    auto ma = CreateTestMarker();

    namespace sviz = smpl::visual;

    sviz::set_visualization_level(SV_NAME_PREFIX ".debug", sviz::Level::Debug);
    sviz::set_visualization_level(SV_NAME_PREFIX ".info", sviz::Level::Info);
    sviz::set_visualization_level(SV_NAME_PREFIX ".warn", sviz::Level::Warn);
    sviz::set_visualization_level(SV_NAME_PREFIX ".error", sviz::Level::Error);
    sviz::set_visualization_level(SV_NAME_PREFIX ".fatal", sviz::Level::Fatal);

    ma.ns = "debug";
    SV_SHOW_DEBUG_COND_NAMED("debug", true, ma);
    SV_SHOW_INFO_COND_NAMED("debug", true, ma);
    SV_SHOW_WARN_COND_NAMED("debug", true, ma);
    SV_SHOW_ERROR_COND_NAMED("debug", true, ma);
    SV_SHOW_FATAL_COND_NAMED("debug", true, ma);

    ma.ns = "info";
    SV_SHOW_DEBUG_COND_NAMED("info", true, ma);
    SV_SHOW_INFO_COND_NAMED("info", true, ma);
    SV_SHOW_WARN_COND_NAMED("info", true, ma);
    SV_SHOW_ERROR_COND_NAMED("info", true, ma);
    SV_SHOW_FATAL_COND_NAMED("info", true, ma);

    ma.ns = "warn";
    SV_SHOW_DEBUG_COND_NAMED("warn", true, ma);
    SV_SHOW_INFO_COND_NAMED("warn", true, ma);
    SV_SHOW_WARN_COND_NAMED("warn", true, ma);
    SV_SHOW_ERROR_COND_NAMED("warn", true, ma);
    SV_SHOW_FATAL_COND_NAMED("warn", true, ma);

    ma.ns = "error";
    SV_SHOW_DEBUG_COND_NAMED("error", true, ma);
    SV_SHOW_INFO_COND_NAMED("error", true, ma);
    SV_SHOW_WARN_COND_NAMED("error", true, ma);
    SV_SHOW_ERROR_COND_NAMED("error", true, ma);
    SV_SHOW_FATAL_COND_NAMED("error", true, ma);

    ma.ns = "fatal";
    SV_SHOW_DEBUG_COND_NAMED("fatal", true, ma);
    SV_SHOW_INFO_COND_NAMED("fatal", true, ma);
    SV_SHOW_WARN_COND_NAMED("fatal", true, ma);
    SV_SHOW_ERROR_COND_NAMED("fatal", true, ma);
    SV_SHOW_FATAL_COND_NAMED("fatal", true, ma);
}

void TestNamedOnceMacros()
{
    auto ma = CreateTestMarker();

    namespace sviz = smpl::visual;

    sviz::set_visualization_level(SV_NAME_PREFIX ".debug", sviz::Level::Debug);
    sviz::set_visualization_level(SV_NAME_PREFIX ".info", sviz::Level::Info);
    sviz::set_visualization_level(SV_NAME_PREFIX ".warn", sviz::Level::Warn);
    sviz::set_visualization_level(SV_NAME_PREFIX ".error", sviz::Level::Error);
    sviz::set_visualization_level(SV_NAME_PREFIX ".fatal", sviz::Level::Fatal);

    ma.ns = "debug";
    SV_SHOW_DEBUG_ONCE_NAMED("debug", ma);
    SV_SHOW_INFO_ONCE_NAMED("debug", ma);
    SV_SHOW_WARN_ONCE_NAMED("debug", ma);
    SV_SHOW_ERROR_ONCE_NAMED("debug", ma);
    SV_SHOW_FATAL_ONCE_NAMED("debug", ma);

    ma.ns = "info";
    SV_SHOW_DEBUG_ONCE_NAMED("info", ma);
    SV_SHOW_INFO_ONCE_NAMED("info", ma);
    SV_SHOW_WARN_ONCE_NAMED("info", ma);
    SV_SHOW_ERROR_ONCE_NAMED("info", ma);
    SV_SHOW_FATAL_ONCE_NAMED("info", ma);

    ma.ns = "warn";
    SV_SHOW_DEBUG_ONCE_NAMED("warn", ma);
    SV_SHOW_INFO_ONCE_NAMED("warn", ma);
    SV_SHOW_WARN_ONCE_NAMED("warn", ma);
    SV_SHOW_ERROR_ONCE_NAMED("warn", ma);
    SV_SHOW_FATAL_ONCE_NAMED("warn", ma);

    ma.ns = "error";
    SV_SHOW_DEBUG_ONCE_NAMED("error", ma);
    SV_SHOW_INFO_ONCE_NAMED("error", ma);
    SV_SHOW_WARN_ONCE_NAMED("error", ma);
    SV_SHOW_ERROR_ONCE_NAMED("error", ma);
    SV_SHOW_FATAL_ONCE_NAMED("error", ma);

    ma.ns = "fatal";
    SV_SHOW_DEBUG_ONCE_NAMED("fatal", ma);
    SV_SHOW_INFO_ONCE_NAMED("fatal", ma);
    SV_SHOW_WARN_ONCE_NAMED("fatal", ma);
    SV_SHOW_ERROR_ONCE_NAMED("fatal", ma);
    SV_SHOW_FATAL_ONCE_NAMED("fatal", ma);
}

void TestHierarchicalLevels()
{
    namespace sviz = smpl::visual;

    sviz::set_visualization_level(SV_NAME_PREFIX ".a", sviz::Level::Debug);
    sviz::set_visualization_level(SV_NAME_PREFIX ".a.b", sviz::Level::Warn);
    sviz::set_visualization_level(SV_NAME_PREFIX ".a.c.d", sviz::Level::Info);

    auto m = CreateTestMarker();

    // a is set to debug

    m.ns = "a";
    SV_SHOW_DEBUG_NAMED("a", m);

    m.ns = "a";
    SV_SHOW_INFO_NAMED("a", m);

    // b overrides a to warning

    m.ns = "a.b";
    SV_SHOW_DEBUG_NAMED("a.b", m);

    m.ns = "a.b";
    SV_SHOW_WARN_NAMED("a.b", m);

    // c inherits a's debug level

    m.ns = "a.c";
    SV_SHOW_DEBUG_NAMED("a.c", m);

    m.ns = "a.c";
    SV_SHOW_INFO_NAMED("a.c", m);

    // d overrides c's inherited level of debug to info
    m.ns = "a.c.d";
    SV_SHOW_DEBUG_NAMED("a.c.d", m);

    m.ns = "a.c.d";
    SV_SHOW_INFO_NAMED("a.c.d", m);
}

int main(int argc, char* argv[])
{
    // features:
    // 1. replaceable visualizer
    // 2. throttled visualizations
    // 3. once visualizations
    // 4. named visualizations
    // 5. hierarchical channel levels
    // 6. conditional visualizations
    // 7. compile-time removal of visualizations
    // 8. runtime channel level reconfiguration
    // 9. severity hierarchy

    TextVisualizer visual;
    smpl::visual::set_visualizer(&visual);

    printf("---vanilla---\n");
    TestBareMacros();
    printf("---\n");

    printf("---cond---\n");
    TestCondMacros();
    printf("---\n");

    printf("---once---\n");
    TestOnceMacros();
    printf("---\n");

    printf("---throttle---\n");
    TestThrottleMacros();
    printf("---\n");

    printf("---named---\n");
    TestNamedMacros();
    printf("---\n");

    printf("---named cond---\n");
    TestNamedCondMacros();
    printf("---\n");

    printf("---named once---\n");
    TestNamedOnceMacros();
    printf("---\n");

    printf("---hierarchical levels---\n");
    TestHierarchicalLevels();
    printf("---\n");

    return 0;
}
