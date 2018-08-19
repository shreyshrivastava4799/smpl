#include <smpl/debug/visualize.h>

#include <fstream>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <boost/program_options.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#ifdef SMPL_HAS_VISUALIZATION_MSGS
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#endif

#include <smpl/stl/memory.h>
#include <smpl/console/console.h>

namespace smpl {
namespace visual {

static auto to_cstring(Level level) -> const char*
{
    switch (level) {
    case Level::Debug:  return "DEBUG";
    case Level::Info:   return "INFO";
    case Level::Warn:   return "WARN";
    case Level::Error:  return "ERROR";
    case Level::Fatal:  return "FATAL";
    default:            return "<UNRECOGNIZED>";
    }
}

// Implementation of the channel hierarchy
namespace impl {

////////////////////////////
// Channel Implementation //
////////////////////////////

struct Channel
{
    std::unique_ptr<Channel>    child = NULL;
    std::unique_ptr<Channel>    next = NULL;
    std::string                 name;
    Level                       severity = Level::Info;
    bool                        inherit = true;

    Channel() = default;
    Channel(std::string name) : name(std::move(name)) { }
};

static
auto FindOrCreateChannel(Channel* channel, const std::string& fullname)
    -> Channel*
{
    SMPL_DEBUG_NAMED("__visual", "Find visualizer %s", fullname.c_str());
    std::vector<std::string> names;
    boost::algorithm::split(
            names,
            fullname,
            boost::is_any_of("."),
            boost::token_compress_on);

    // need at least the root name
    assert(!names.empty());

    // skip the first name, assuming it is the root visualizer name
    auto nit = begin(names);
    assert(*nit == SV_ROOT_VIZ_NAME);

    ++nit;

    auto* visualizer = channel;
    for (; nit != end(names); ++nit) {
        auto& name = *nit;
        // find or create a child that matches the name

        if (visualizer->child) { // we have at least one child
            auto found = false;
            auto* child = visualizer->child.get(); // start with the first child
            for (;;) {
                // break if this child matches the name
                if (child->name == name) {
                    visualizer = child;
                    found = true;
                    break;
                }

                if (child->next == NULL) { // the last child
                    break;
                } else {
                    child = child->next.get();
                }
            }

            if (!found) {
                // create a new child
                SMPL_DEBUG_NAMED("__visual", " -> Create new visualizer %s", name.c_str());
                child->next = make_unique<Channel>();
                child->next->name = name;
                child->next->severity = visualizer->severity;
                visualizer = child->next.get();

                // and, recurse!
            } else {
                // continue searching on the tree rooted at the child
                visualizer = child;
            }
        } else { // has no children
            // create the first child
            SMPL_DEBUG_NAMED("__visual", " -> Create new visualizer %s", name.c_str());
            visualizer->child = make_unique<Channel>();
            visualizer->child->name = name;
            visualizer->child->severity = visualizer->severity;

            // TODO: do we need to create intermediate children with
            // default/inherited levels?

            // recurse
            visualizer = visualizer->child.get();
        }
    }

    return visualizer;
}

////////////////////////
// Channels Interface //
////////////////////////

// The set of hierarchical channels, implemented as a tree of channels
using Channels = std::unique_ptr<Channel>;

auto MakeChannels() -> Channels
{
    return make_unique<Channel>(SV_ROOT_VIZ_NAME);
}

static
void* GetChannelHandle(Channels* channel, const std::string& fullname)
{
    return FindOrCreateChannel(channel->get(), fullname);
}

static
bool UpdateChannelLevel(
    Channels* channel,
    const std::string& fullname,
    Level level)
{
    SMPL_DEBUG_NAMED("__visual", "Update level of channel '%s' to '%s'", fullname.c_str(), to_cstring(level));
    auto* visualizer = FindOrCreateChannel(channel->get(), fullname);

    if (visualizer->severity == level) {
        SMPL_DEBUG_NAMED("__visual", " -> detach from parent");
        visualizer->inherit = false;
        return false;
    }

    // set the level of this visualizer
    visualizer->severity = level;
    visualizer->inherit = false;

    // propagate channel severity to children

    std::vector<Channel*> channels;

    for (auto* child = visualizer->child.get();
        child != NULL;
        child = child->next.get())
    {
        if (child->inherit) {
            SMPL_DEBUG_NAMED("__visual", " -> propagate to child '%s'", child->name.c_str());
            channels.push_back(child);
        }
    }

    while (!channels.empty()) {
        auto* channel = channels.back();
        channels.pop_back();

        // update severity
        channel->severity = level;

        // recurse on non-overridden children
        for (auto* child = channel->child.get();
            child != NULL;
            child = child->next.get())
        {
            if (child->inherit) {
                SMPL_DEBUG_NAMED("__visual", " -> propagate to child '%s'", child->name.c_str());
                channels.push_back(child);
            }
        }
    }

    return true;
}

static
bool IsChannelEnabledFor(Channel* handle, Level level)
{
    return level >= handle->severity;
}

static
void GetChannels(
    Channel* channel,
    const std::string& prefix,
    std::unordered_map<std::string, Level>& out)
{
    auto fullname = prefix + "." + channel->name;
    out[fullname] = channel->severity;
    for (auto* child = channel->child.get();
        child != NULL;
        child = child->next.get())
    {
        GetChannels(child, fullname, out);
    }
}

static
void GetChannels(
    Channels* channels,
    std::unordered_map<std::string, Level>& out)
{
    auto* root = channels->get();

    out[root->name] = root->severity;

    for (auto* child = root->child.get();
        child != NULL;
        child = child->next.get())
    {
        GetChannels(child, root->name, out);
    }
}

} // namespace impl

//////////////////////////////////////////
// Global Channel System Implementation //
//////////////////////////////////////////

// global channel hierarchy
static impl::Channels g_channels = impl::MakeChannels();

// global flag to trigger initialization of channel levels from config file
static bool g_initialized = false;

static
void Initialize()
{
    if (g_initialized) {
        return;
    }

    auto* config_path = getenv("SMPL_VISUALIZE_CONFIG_FILE");
    if (config_path == NULL) {
        g_initialized = true;
        return;
    }

    namespace po = boost::program_options;

    po::options_description ops;

    bool allow_unregistered = true;
    auto pops = po::parse_config_file<char>(config_path, ops, allow_unregistered);

    po::variables_map vm;
    po::store(pops, vm);
    po::notify(vm);

    for (auto& op : pops.options) {
        if (op.unregistered) {
            auto& levelstr = op.value.back();
            Level level = Level::NumLevels;
            if (levelstr == "INFO") {
                level = Level::Info;
            } else if (levelstr == "DEBUG") {
                level = Level::Debug;
            } else if (levelstr == "WARN") {
                level = Level::Warn;
            } else if (levelstr == "ERROR") {
                level = Level::Error;
            } else if (levelstr == "FATAL") {
                level = Level::Fatal;
            }

            if (level != Level::NumLevels) {
                UpdateChannelLevel(&g_channels, op.string_key, level);
            }
        }
    }

    g_initialized = true;
}

static
void* GetHandle(const std::string& name)
{
    Initialize();
    return GetChannelHandle(&g_channels, name);
}

static
bool IsEnabledFor(void* handle, Level level)
{
    Initialize();
    return IsChannelEnabledFor(static_cast<impl::Channel*>(handle), level);
}

static
void GetVisualizations(std::unordered_map<std::string, Level>& visualizations)
{
    Initialize();
    visualizations.clear();
    GetChannels(&g_channels, visualizations);
}

static
bool SetVisualizationLevel(const std::string& name, Level level)
{
    Initialize();
    return UpdateChannelLevel(&g_channels, name, level);
}

/////////////////////////////////////
// Global Channel System Interface //
/////////////////////////////////////

static void NotifyLevelsChanged();

// thread-safe initialization of debug visualization locations, defined here
// since the channel system needs to lock all locations while modifying the
// channel hierarchy.
static std::mutex g_locations_mutex;

void get_visualizations(std::unordered_map<std::string, Level>& visualizations)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);
    GetVisualizations(visualizations);
}

bool set_visualization_level(const std::string& name, Level level)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);
    bool changed = SetVisualizationLevel(name, level);
    if (changed) {
        NotifyLevelsChanged();
    }
    return changed;
}

///////////////////////////////////////////////////
// Visualization Locations System Implementation //
///////////////////////////////////////////////////

// global singly-linked list of locations, for batch updates when visualization
// levels change
static VizLocation* g_loc_head = nullptr;
static VizLocation* g_loc_tail = nullptr;

static
void UpdateLocationEnabledNoLock(VizLocation* loc)
{
    loc->enabled = IsEnabledFor(loc->handle, loc->level);
}

void InitializeVizLocation(
    VizLocation* loc,
    const std::string& name,
    Level level)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);

    if (loc->initialized) {
        return;
    }

    loc->handle = GetHandle(name);
    loc->level = level;

    if (!g_loc_head) {
        // list is empty
        assert(!g_loc_tail);
        g_loc_head = loc;
        g_loc_tail = loc;
    } else {
        g_loc_tail->next = loc;
        g_loc_tail = loc;
    }

    UpdateLocationEnabledNoLock(loc);

    loc->initialized = true;
}

static
void NotifyLevelsChanged()
{
    for (VizLocation *loc = g_loc_head; loc != NULL; loc = loc->next) {
        UpdateLocationEnabledNoLock(loc);
    }
}

// NOTE: The Visualization Locations System Interface is defined by the
// visualization macros

///////////////////////////////////////////
// Default VisualizerBase Implementation //
///////////////////////////////////////////

#ifdef SMPL_HAS_VISUALIZATION_MSGS
void VisualizerBase::visualize(
    Level level,
    const visualization_msgs::Marker& mm)
{
}

void VisualizerBase::visualize(
    Level level,
    const visualization_msgs::MarkerArray& markers)
{
    for (auto& m : markers.markers) {
        visualize(level, m);
    }
}
#endif

void VisualizerBase::visualize(
    Level level,
    const std::vector<visual::Marker>& markers)
{
    for (auto& m : markers) {
        visualize(level, m);
    }
}

void VisualizerBase::visualize(
    Level level,
    const visual::Marker& marker)
{
}

/////////////////////////////////////
// Global VisualizerBase Interface //
/////////////////////////////////////

// global visualizer
static VisualizerBase* g_visualizer;

// guards calls to global visualizer's visualize() method
static std::mutex g_viz_mutex;

void set_visualizer(VisualizerBase* visualizer)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    g_visualizer = visualizer;
}

void unset_visualizer()
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    g_visualizer = nullptr;
}

auto visualizer() -> VisualizerBase*
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    return g_visualizer;
}

void visualize(Level level, const visual::Marker& marker)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    if (!g_visualizer) {
        return;
    }

    g_visualizer->visualize(level, marker);
}

void visualize(Level level, const std::vector<visual::Marker>& markers)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    if (!g_visualizer) {
        return;
    }

    g_visualizer->visualize(level, markers);
}

#ifdef SMPL_HAS_VISUALIZATION_MSGS
void visualize(Level level, const visualization_msgs::Marker& marker)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    if (!g_visualizer) {
        return;
    }

    g_visualizer->visualize(level, marker);
}

void visualize(Level level, const visualization_msgs::MarkerArray& markers)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    if (!g_visualizer) {
        return;
    }

    g_visualizer->visualize(level, markers);
}
#endif

} // namespace viz
} // namespace smpl
