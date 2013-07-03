#include <sbpl_collision_checking/group.h>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <geometric_shapes/shapes.h>

#define RESOLUTION 0.02

namespace sbpl_arm_planner
{

bool sortSphere(sbpl_arm_planner::Sphere* a, sbpl_arm_planner::Sphere* b)
{
  return a->priority < b->priority;
}

Group::Group(std::string name) : name_(name)
{
  init_ = false;
}

Group::~Group()
{
  for(std::size_t i = 0; i < solvers_.size(); i++)
  {
    if(solvers_[i] != NULL)
    {
      delete solvers_[i];
      solvers_[i] = NULL;
    }
  }
}

void Group::print()
{
  if(!init_)
  {
    ROS_ERROR("Failed to print %s group information because has not yet been initialized.", name_.c_str());
    return;
  }

  ROS_INFO("name: %s", name_.c_str());
  ROS_INFO("type: %d", type_);
  ROS_INFO("root name: %s", root_name_.c_str());
  ROS_INFO(" tip name: %s", tip_name_.c_str());
  ROS_INFO("collision links: ");
  for(std::size_t i = 0; i < links_.size(); ++i)
  {
    ROS_INFO(" name: %s", links_[i].name_.c_str());
    ROS_INFO(" root: %s", links_[i].root_name_.c_str());
    if(type_ == sbpl_arm_planner::Group::SPHERES)
    {
      ROS_INFO(" spheres: %d", int(links_[i].spheres_.size()));
      for(std::size_t j = 0; j < links_[i].spheres_.size(); ++j)
        ROS_INFO("  [%s] x: %0.3f y: %0.3f z: %0.3f radius: %0.3f priority: %d", links_[i].spheres_[j].name.c_str(), links_[i].spheres_[j].v.x(), links_[i].spheres_[j].v.y(), links_[i].spheres_[j].v.z(), links_[i].spheres_[j].radius, links_[i].spheres_[j].priority);
    }
    else if(type_ == sbpl_arm_planner::Group::VOXELS)
    {
      ROS_INFO(" voxels: %d", int(links_[i].voxels_.v.size()));
      for(std::size_t j = 0; j < links_[i].voxels_.v.size(); ++j)
        ROS_DEBUG("  [%d] x: %0.3f y: %0.3f z: %0.3f", int(j), links_[i].voxels_.v[j].x(), links_[i].voxels_.v[j].y(), links_[i].voxels_.v[j].z());
    }
    if(i < links_.size()-1)
      ROS_INFO(" ---");
  }
  ROS_INFO(" ");
  if(type_ == sbpl_arm_planner::Group::SPHERES)
  {
    ROS_INFO("sorted spheres: ");
    for(std::size_t j = 0; j < spheres_.size(); ++j)
    {
      ROS_INFO("  [%s] x: %0.3f  y:%0.3f  z:%0.3f  radius: %0.3f  priority: %d", spheres_[j]->name.c_str(), spheres_[j]->v.x(), spheres_[j]->v.y(), spheres_[j]->v.z(), spheres_[j]->radius, spheres_[j]->priority);
    }
    ROS_INFO(" ");
  }
  ROS_INFO("kinematic chain(s): ");
  for(std::size_t j = 0; j < chains_.size(); ++j)
    leatherman::printKDLChain(chains_[j], "chain " + boost::lexical_cast<std::string>(j));
  ROS_INFO(" ");
}

bool Group::init(boost::shared_ptr<urdf::Model> urdf)
{
  if(!initKinematics())
    return false;

  if(type_ == sbpl_arm_planner::Group::VOXELS)
    init_ = initVoxels();
  else
    init_ = initSpheres();
  
  return init_;
}

bool Group::initKinematics()
{
  bool unincluded_links = true;
  int cnt = 0;
  std::vector<int> link_included(links_.size(),-1);
  KDL::Chain chain;
  KDL::Tree tree;

  if (!kdl_parser::treeFromUrdfModel(*urdf_, tree))
  {
    ROS_ERROR("Failed to parse tree from robot description.");
    return false;
  }

  // loop until all links are included in a single kdl chain
  while(unincluded_links && cnt < 100)
  {
    cnt++;
    std::vector<int> num_links_per_tip(links_.size(),0);

    // compute # of links each link would include if tip of chain
    for(size_t i = 0; i < links_.size(); ++i)
    {
      // link i is already included in a chain
      if(link_included[i] > -1)
        continue;

      // create chain with link i as the tip
      if (!tree.getChain(root_name_, links_[i].root_name_, chain))
      {
        ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting. (root: %s, tip: %s)", root_name_.c_str(), links_[i].root_name_.c_str());
        continue;
      }

      // count number of links included in this chain
      for(size_t j = 0; j < links_.size(); ++j)
      {
        // link j is already included in a chain
        if(link_included[j] > -1)
          continue;

        // check if link is included in this chain
        int seg;
        if(leatherman::getSegmentIndex(chain, links_[j].root_name_, seg))
          num_links_per_tip[i]++;
      }
    }

    // get chain tip that would include the most links
    int i_max = 0;
    for(size_t i = 0; i < num_links_per_tip.size(); ++i)
    {
      ROS_DEBUG("chain_tip: %25s  included_links %d", links_[i].root_name_.c_str(), num_links_per_tip[i]);
      if(num_links_per_tip[i] > num_links_per_tip[i_max])
        i_max = i;
    }
    ROS_INFO("Creating a chain for %s group with %s as the tip.", name_.c_str(), links_[i_max].root_name_.c_str());

    // create chain with link i_max as the tip
    if (!tree.getChain(root_name_, links_[i_max].root_name_, chain))
    {
      ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting. (root: %s, tip: %s)", root_name_.c_str(), links_[i_max].root_name_.c_str());
      continue;
    }

    // add chain to the group
    chains_.push_back(chain);

    // mark links that are included in this chain
    int included_links = 0;
    for(size_t i = 0; i < links_.size(); ++i)
    {
      // link i is already included in a different chain
      if(link_included[i] > -1)
      {
        included_links++;
        continue;
      }

      // check if link i is included in this chain
      int seg;
      if(leatherman::getSegmentIndex(chains_.back(), links_[i].root_name_, seg))
      {
        link_included[i] = chains_.size()-1;
        links_[i].i_chain_ = chains_.size()-1;
        included_links++;
        ROS_DEBUG("[chain: %s] [%d] includes: %s", links_[i_max].root_name_.c_str(), included_links, links_[i].root_name_.c_str());
      }
    }

    if(included_links == int(links_.size()))
      unincluded_links = false;

    ROS_DEBUG("Completed %d loops of the while loop (included_links = %d)", cnt, included_links);
  }

  for(size_t i = 0; i < link_included.size(); ++i)
    ROS_DEBUG("included link: %25s  link_root: %25s  chain: %d", links_[i].name_.c_str(), links_[i].root_name_.c_str(), link_included[i]);

  if(cnt >= 100)
    return false;

  // initialize the FK solvers
  solvers_.resize(chains_.size());
  for(size_t i = 0; i < chains_.size(); ++i)
  {
    solvers_[i] = new KDL::ChainFkSolverPos_recursive(chains_[i]);
    ROS_INFO("[%s] Instantiated a forward kinematics solver for chain #%d for the %s with %d joints.", name_.c_str(), int(i), name_.c_str(), chains_[i].getNrOfJoints());
  }

  ROS_INFO("Initialized %d chains for the %s group.", int(chains_.size()), name_.c_str());
  return true;
}

/*
void Group::getParams()
{
  ros::NodeHandle ph_("~");
  XmlRpc::XmlRpcValue all_groups;

  std::string group_name = "collision_groups";

  if(!ph_.hasParam(group_name)) 
  {
    ROS_WARN_STREAM("No groups for planning specified in " << group_name);
    return;
  }
  ph_.getParam(group_name, all_groups);

  if(all_groups.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    ROS_WARN("Groups is not an array.");

  if(all_groups.size() == 0) 
  {
    ROS_WARN("No groups in groups");
    return;
  }

  for(int i = 0; i < all_groups.size(); i++) 
  {
    if(!all_groups[i].hasMember("name"))
    {
      ROS_WARN("All groups must have a name.");
      continue;
    }
    std::string gname = all_groups[i]["name"];
    Group* gc = new Group(gname);
    gc->init_ = false;
    std::map< std::string, Group*>::iterator group_iterator = group_config_map_.find(gname);
    if(group_iterator != group_config_map_.end())
    {
      ROS_WARN_STREAM("Already have group name " << gname);
      delete gc;
      continue;
    }
    group_config_map_[gname] = gc;

    if(!all_groups[i].hasMember("type"))
    {
      ROS_WARN("All groups must have a type. (voxels or spheres)");
      continue;
    }
    if(std::string(all_groups[i]["type"]).compare("voxels") == 0)
      group_config_map_[gname]->type_ = sbpl_arm_planner::Group::VOXELS;
    else if(std::string(all_groups[i]["type"]).compare("spheres") == 0)
      group_config_map_[gname]->type_ = sbpl_arm_planner::Group::SPHERES;
    else
    {
      ROS_ERROR("Illegal group type. (voxels or spheres)");
      continue;
    }

    if(!all_groups[i].hasMember("root_name"))
    {
      ROS_WARN("All groups must have a root_name.");
      continue;
    }
    group_config_map_[gname]->root_name_ = std::string(all_groups[i]["root_name"]);

    if(!all_groups[i].hasMember("tip_name"))
    {
      ROS_WARN("All groups must have a tip_name.");
      continue;
    }
    group_config_map_[gname]->tip_name_ = std::string(all_groups[i]["tip_name"]);

    if(all_groups[i].hasMember("collision_links"))
    {
      XmlRpc::XmlRpcValue all_links = all_groups[i]["collision_links"];

      if(all_links.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_WARN("Collision links is not an array.");
        return;
      }

      if(all_links.size() == 0) 
      {
        ROS_WARN("No links in collision links");
        return;
      }

      Link link;
      for(int j = 0; j < all_links.size(); j++) 
      {
        link.spheres_.clear();
        if(all_links[j].hasMember("name"))
          link.name_ = std::string(all_links[j]["name"]);
        if(all_links[j].hasMember("root"))
          link.root_name_ = std::string(all_links[j]["root"]);
        else
          ROS_WARN("No root name");

        if(group_config_map_[gname]->type_ == sbpl_arm_planner::Group::SPHERES)
        {
          std::stringstream ss(all_links[j]["spheres"]);
          Sphere sphere;
          double x,y,z;
          std::vector<Sphere> link_spheres;
          std::string sphere_name;
          while(ss >> sphere_name)
          {
            ros::NodeHandle sphere_node(ph_, sphere_name);
            sphere.name = sphere_name;
            sphere_node.param("x", x, 0.0);
            sphere_node.param("y", y, 0.0);
            sphere_node.param("z", z, 0.0);
            sphere_node.param("radius", sphere.radius, 0.0);
            sphere_node.param("priority", sphere.priority, 1);
            sphere.v.x(x);
            sphere.v.y(y);
            sphere.v.z(z);
            link.spheres_.push_back(sphere);
          }
        }
        group_config_map_[gname]->links_.push_back(link);
      }
    }
  }
  ROS_INFO("Successfully parsed collision model");
}
*/

void Group::printSpheres()
{
  if(!init_)
  {
    ROS_ERROR("Failed to print the collision spheres because the %s group is not initialized.", name_.c_str());
    return;
  }

  ROS_INFO("\n%s", name_.c_str());
  for(size_t i = 0; i < spheres_.size(); ++i)
  {
    ROS_INFO("[%s] x: %0.3f  y:%0.3f  z:%0.3f  radius: %0.3f  priority: %d", spheres_[i]->name.c_str(), spheres_[i]->v.x(), spheres_[i]->v.y(), spheres_[i]->v.z(), spheres_[i]->radius, spheres_[i]->priority);
  }
  ROS_INFO(" ");
}

bool Group::initSpheres()
{
  if(type_ == sbpl_arm_planner::Group::VOXELS)
    return false;

  // assign the kdl segment numbers to each sphere
  for(size_t i = 0; i < links_.size(); ++i)
  {
    int seg;
    if(!leatherman::getSegmentIndex(chains_[links_[i].i_chain_], links_[i].root_name_, seg))
      return false;

    for(size_t j = 0; j < links_[i].spheres_.size(); ++j)
    {
      links_[i].spheres_[j].kdl_segment = seg;
      links_[i].spheres_[j].kdl_chain = links_[i].i_chain_;
    }
  }

  // fill the group's list of all the spheres
  for(size_t i = 0; i < links_.size(); ++i)
  {
    for(size_t j = 0; j < links_[i].spheres_.size(); ++j)
      spheres_.push_back(&(links_[i].spheres_[j]));
  }

  // sort the spheres by priority
  sort(spheres_.begin(), spheres_.end(), sortSphere);

  // populate the frames vector that stores the segments in each chain 
  frames_.resize(chains_.size());
  for(size_t i = 0; i < spheres_.size(); ++i)
  {
    if(std::find(frames_[spheres_[i]->kdl_chain].begin(), frames_[spheres_[i]->kdl_chain].end(), spheres_[i]->kdl_segment) == frames_[spheres_[i]->kdl_chain].end())
      frames_[spheres_[i]->kdl_chain].push_back(spheres_[i]->kdl_segment);
  }

  // debug output
  ROS_DEBUG("[%s] Frames:", name_.c_str());
  for(size_t i = 0; i < frames_.size(); ++i)
    for(size_t j = 0; j < frames_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] segment: %d", name_.c_str(), int(i), frames_[i][j]);

  // get order of joints for each chain
  jntarray_names_.resize(chains_.size());
  for(size_t i = 0; i < chains_.size(); ++i)
  {
    for(size_t j = 0; j < chains_[i].getNrOfSegments(); ++j)
    {
      if(chains_[i].getSegment(j).getJoint().getTypeName().compare("None") != 0)
      {
        jntarray_names_[i].push_back(chains_[i].getSegment(j).getJoint().getName());
      }
    }
  }

  // debug output
  ROS_DEBUG("[%s] Order of Joints in Joint Array input to the FK Solver:", name_.c_str());
  for(size_t i = 0; i < jntarray_names_.size(); ++i)
    for(size_t j = 0; j < jntarray_names_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] %d: %s", name_.c_str(), int(i), int(j), jntarray_names_[i][j].c_str());

  // initialize the sizes of the JntArrays
  joint_positions_.resize(chains_.size());
  for(size_t i = 0; i < chains_.size(); ++i)
  {
    joint_positions_[i].resize(jntarray_names_[i].size());
    KDL::SetToZero(joint_positions_[i]);
  }
  return true;
}

bool Group::initVoxels()
{
  // get link voxels && assign the kdl segment numbers to each link 
  for(size_t i = 0; i < links_.size(); ++i)
  {
    if(!getLinkVoxels(links_[i].root_name_, links_[i].voxels_.v))
    {
      ROS_ERROR("Failed to retrieve voxels for link '%s' in group '%s'", links_[i].root_name_.c_str(), name_.c_str());
      return false;
    }
    ROS_DEBUG("Retrieved %d voxels for link '%s'", int(links_[i].voxels_.v.size()), links_[i].root_name_.c_str());
    int seg;
    if(!leatherman::getSegmentIndex(chains_[links_[i].i_chain_], links_[i].root_name_, seg))
      return false;

    links_[i].voxels_.kdl_segment = seg;
    links_[i].voxels_.kdl_chain = links_[i].i_chain_;
  }

  // populate the frames vector that stores the segments in each chain 
  frames_.resize(chains_.size());
  for(size_t i = 0; i < links_.size(); ++i)
    frames_[links_[i].voxels_.kdl_chain].push_back(links_[i].voxels_.kdl_segment);

  // debug output
  ROS_DEBUG("[%s] Frames:", name_.c_str());
  for(size_t i = 0; i < frames_.size(); ++i)
    for(size_t j = 0; j < frames_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] segment: %d", name_.c_str(), int(i), frames_[i][j]);


  // get order of joints for each chain
  jntarray_names_.resize(chains_.size());
  for(size_t i = 0; i < chains_.size(); ++i)
  {
    for(size_t j = 0; j < chains_[i].getNrOfSegments(); ++j)
    {
      if(chains_[i].getSegment(j).getJoint().getTypeName().compare("None") != 0)
      {
        jntarray_names_[i].push_back(chains_[i].getSegment(j).getJoint().getName());
      }
    }
  }

  // debug output
  ROS_DEBUG("[%s] Order of Joints in Joint Array input to the FK Solver:", name_.c_str());
  for(size_t i = 0; i < jntarray_names_.size(); ++i)
    for(size_t j = 0; j < jntarray_names_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] %d: %s", name_.c_str(), int(i), int(j), jntarray_names_[i][j].c_str());

  // initialize the sizes of the JntArrays
  joint_positions_.resize(chains_.size());
  for(size_t i = 0; i < chains_.size(); ++i)
  {
    joint_positions_[i].resize(jntarray_names_[i].size());
    KDL::SetToZero(joint_positions_[i]);
  }
  return true;
}

bool Group::computeFK(const std::vector<double> &angles, int chain, int segment, KDL::Frame &frame)
{
  // sort elements of input angles into proper positions in the JntArray
  for(size_t i = 0; i < angles.size(); ++i)
  {
    if(angles_to_jntarray_[chain][i] == -1)
      continue;
    joint_positions_[chain](angles_to_jntarray_[chain][i]) = angles[i];
  }

  if(solvers_[chain]->JntToCart(joint_positions_[chain], frame, segment) < 0)
  {
    ROS_ERROR("JntToCart returned < 0. Exiting.");
    return false;
  }
  return true;
}

void Group::setOrderOfJointPositions(const std::vector<std::string> &joint_names)
{
  // store the desired order of the input angles for debug information
  order_of_input_angles_ = joint_names;

  // for each joint, find its proper index in the JntArray for each chain's solver
  angles_to_jntarray_.resize(chains_.size());
  for(size_t i = 0; i < joint_names.size(); ++i)
  {
    bool matched = false; // kinda useless

    ROS_DEBUG("[%s] [%d] %s", name_.c_str(), int(i), joint_names[i].c_str());
    for(size_t k = 0; k < chains_.size(); ++k)
    {
      angles_to_jntarray_[k].resize(joint_names.size(),-1);
      for(size_t j = 0; j < jntarray_names_[k].size(); ++j)
      {
        if(joint_names[i].compare(jntarray_names_[k][j]) == 0)
        {
          angles_to_jntarray_[k][i] = j;
          matched = true;
          break;
        }
      }
    }
    if(!matched)
      ROS_ERROR("%s was not found in either chain. Why do you send it to the forward kinematics solver?", joint_names[i].c_str());
  }

  for(size_t i = 0; i < angles_to_jntarray_.size(); ++i)
    for(size_t j = 0; j < angles_to_jntarray_[i].size(); ++j)
      ROS_DEBUG("[%s] [chain %d] joint: %s  index: %d", name_.c_str(), int(i), joint_names[j].c_str(), angles_to_jntarray_[i][j]);
}

void Group::setJointPosition(const std::string &name, double position)
{
  for(std::size_t i = 0; i < jntarray_names_.size(); ++i)
  {
    for(std::size_t j = 0; j < jntarray_names_[i].size(); ++j)
    {
      if(name.compare(jntarray_names_[i][j]) == 0)
      {
        joint_positions_[i](j) = position;
        break;
      }
    }
  }
}

std::string Group::getReferenceFrame()
{
  if(!init_)
    return "";
  return root_name_;
}

bool Group::getLinkVoxels(std::string name, std::vector<KDL::Vector> &voxels)
{
  boost::shared_ptr<const urdf::Link> link = urdf_->getLink(name);
  if(link == NULL)
  {
    ROS_ERROR("Failed to find link '%s' in URDF.", name.c_str());
    return false;
  }
  if(link->collision == NULL)
  {
    ROS_ERROR("Failed to find collision field for link '%s' in URDF.", link->name.c_str());
    return false;
  }
  if(link->collision->geometry == NULL)
  {
    ROS_ERROR("Failed to find geometry for link '%s' in URDF. (group: %s)", name.c_str(), link->collision->group_name.c_str());
    return false;
  }

  boost::shared_ptr<const urdf::Geometry> geom = link->collision->geometry;

  if(geom->type == urdf::Geometry::MESH)
  {
    std::vector<int> triangles;
    std::vector<geometry_msgs::Point> vertices;
    std::vector<std::vector<double> > v;
    urdf::Mesh* mesh = (urdf::Mesh*) geom.get();
    if(!leatherman::getMeshComponentsFromResource(mesh->filename, triangles, vertices))
    {
      ROS_ERROR("Failed to get mesh from file. (%s)", mesh->filename.c_str());
      return false;
    }
    ROS_INFO("mesh: %s  triangles: %u  vertices: %u", name.c_str(), int(triangles.size()), int(vertices.size()));
    sbpl::Voxelizer::voxelizeMesh(vertices, triangles, RESOLUTION, v, false, 100000); 
    ROS_INFO("mesh: %s  voxels: %u", name.c_str(), int(v.size()));
    voxels.resize(v.size());
    for(size_t i = 0; i < v.size(); ++i)
    {
      voxels[i].x(v[i][0]); 
      voxels[i].y(v[i][1]); 
      voxels[i].z(v[i][2]); 
    }
  }
  else if(geom->type == urdf::Geometry::BOX)
  {
    std::vector<std::vector<double> > v;
    urdf::Box* box = (urdf::Box*) geom.get();
    sbpl::Voxelizer::voxelizeBox(box->dim.x, box->dim.y, box->dim.z, RESOLUTION, v, false); 
    ROS_INFO("box: %s  voxels: %u", name.c_str(), int(v.size()));
    voxels.resize(v.size());
    for(size_t i = 0; i < v.size(); ++i)
    {
      voxels[i].x(v[i][0]); 
      voxels[i].y(v[i][1]); 
      voxels[i].z(v[i][2]); 
    }
  }
  else if(geom->type == urdf::Geometry::SPHERE)
  {
    std::vector<std::vector<double> > v;
    urdf::Sphere* sph = (urdf::Sphere*) geom.get();
    sbpl::Voxelizer::voxelizeSphere(sph->radius, RESOLUTION, v, false); 
    ROS_INFO("sphere: %s  voxels: %u", name.c_str(), int(v.size()));
    voxels.resize(v.size());
    for(size_t i = 0; i < v.size(); ++i)
    {
      voxels[i].x(v[i][0]); 
      voxels[i].y(v[i][1]); 
      voxels[i].z(v[i][2]); 
    }
  }
  else
  {
    ROS_ERROR("Failed to get voxels for link '%s'.", name.c_str());
    return false;
  }

  return true;
}

}