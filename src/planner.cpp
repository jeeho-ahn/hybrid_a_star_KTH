#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    //subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal_cb, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart_cb, this);

  //plan request service
  srvPlanReqService = n.advertiseService("/plan_req_test", &Planner::plan_req_handler, this);
  //add cube service
  srvCubeReqService = n.advertiseService("/cube_req", &Planner::cube_req_handler, this);
  //Push cost service
  srvPushCostReqService = n.advertiseService("/cube_pickup_cost_req", &Planner::push_cost_req_handler, this);

  //own map server
  map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 2);

  //add cubes on map as obstacles
  srvAddCubesReqService = n.advertiseService("/add_cube_obstacles", &Planner::add_cubes_req_handler, this);

  //todo: parse map file path automatically
  //load_map_yaml("/home/jeeho/catkin_ws/src/hybrid_a_star_KTH/maps/map.yaml");
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = map->info.height;
  int width = map->info.width;
  bool** binMap;
  binMap = new bool*[width];

  //use map origin as cell offset
  auto origin_off = map->info.origin;
  origin_off_x = origin_off.position.x;
  origin_off_y = origin_off.position.y;
  //get map resolution
  //auto resol = map->info.resolution;

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    plan();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################

void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial, bool run_plan) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << initial->pose.pose.position.x << " y:" << initial->pose.pose.position.y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    if (Constants::manual && run_plan) { plan();}

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}


void Planner::setStart_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << initial->pose.pose.position.x << " y:" << initial->pose.pose.position.y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    if (Constants::manual) { plan();}

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal_cb(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << end->pose.position.x << " y:" << end->pose.position.y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end, bool run_plan) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << end->pose.position.x << " y:" << end->pose.position.y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual && run_plan) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
std::shared_ptr<Path> Planner::plan() {
  //verify start and goal
    std::cout << "start x:" << start.pose.pose.position.x << " y:" << start.pose.pose.position.y << " yaw:" << tf::getYaw(start.pose.pose.orientation) << std::endl;
    std::cout << "goal x:" << goal.pose.position.x << " y:" << goal.pose.position.y << " yaw:" << tf::getYaw(goal.pose.orientation) << std::endl;


  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position (Jeeho) add origin offset
    float x = (goal.pose.position.x - origin_off_x) / Constants::cellSize;
    float y = (goal.pose.position.y - origin_off_y) / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position (Jeeho) add origin offset
    x = (start.pose.pose.position.x - origin_off_x) / Constants::cellSize;
    y = (start.pose.pose.position.y - origin_off_y) / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
    if(nSolution == nullptr)
      std::cout << "plan failed" << std::endl;
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath(),origin_off_x,origin_off_y);
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath(),origin_off_x,origin_off_y);
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);

    delete [] nodes3D;
    delete [] nodes2D;

    //return length of path
    //return smoothedPath.getPathLength();
    return std::make_shared<Path>(smoothedPath);

  } else {
    std::cout << "missing goal or start" << std::endl;
    return 0;
  }
}

//###################################################
//                                      PLAN REQUEST
//###################################################
bool Planner::plan_req_handler(hybrid_astar::planReqSrvRequest &req, hybrid_astar::planReqSrvResponse &res)
{
  // add start pose
  auto startPosePtr = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(req.startPose);
  auto goalPosePtr = boost::make_shared<geometry_msgs::PoseStamped>(req.goalPose);
  this->setStart(startPosePtr,false);
  this->setGoal(goalPosePtr, false);

  auto plan_res = plan();
  //todo: return result path, too

  //todo: add failed results
  res.trav_dist.data = plan_res->getPathLength();
  res.planned_path = *(plan_res->get_nav_path());
  res.result.data = false;
  if(res.trav_dist.data > 0)
    res.result.data = true;

  return true;
}



bool Planner::cube_req_handler(hybrid_astar::cubeReqSrvRequest &req, hybrid_astar::cubeReqSrvResponse &res)
{
  std::cout << "Cube Request" << std::endl;
  jeeho::cube temp_cube(req.pose_in);
  if(req.cmd.data == "add")
  {
    cube_list.push_back(temp_cube);
  }
  else if(req.cmd.data == "replace")
  {
    cube_list.clear();
    cube_list.push_back(temp_cube);
  }
  else {
    //invalid cmd
    return false;
  }
  return true;
}

bool Planner::push_cost_req_handler(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  //first cube for now
  if(cube_list.size()>0)
  {
    //force valid
    validGoal = true;
    //get first cost
    int pivot_ind = 0;
    geometry_msgs::PoseStamped pivot_pose;
    //goal orientation candidates
    auto pose_cands = cube_list[0].pose_candidates();
    pivot_pose = pose_cands->at(pivot_ind);
    goal = pivot_pose;
    auto pivot_cost = plan()->getPathLength();
    std::cout << "Cost: " << pivot_cost << std::endl;

    for (size_t i=1;i<4;i++) {
      pivot_pose = pose_cands->at(i);
      goal = pivot_pose;
      float temp_cost = plan()->getPathLength();
      std::cout << "Cost: " << temp_cost << std::endl;
      if(temp_cost<pivot_cost){
        pivot_cost = temp_cost;
        pivot_ind = i;
      }
    }

    pivot_pose = pose_cands->at(pivot_ind);
    goal = pivot_pose;
    auto final_cost = plan();
    std::cout << "Chosen Cost: " << final_cost << std::endl;

  }
  else{
    //cube list is empty
  }

  return true;
}

bool Planner::add_cubes_req_handler(hybrid_astar::addCubesSrvRequest &req, hybrid_astar::addCubesSrvResponse &res)
{
  
  for(size_t n=0; n<req.poses_in.size(); n++)
  {
    // convert to grid coord
    int x_cell = int((req.poses_in[n].position.x - origin_off_x) / Constants::cellSize);
    int y_cell = int((req.poses_in[n].position.y - origin_off_y) / Constants::cellSize);

    // find cells to block and apply
    size_t size_cell = std::ceil(req.cube_size / Constants::cellSize);
    if(size_cell%2 == 0)
      size_cell++;

    auto nCol = grid->info.height;

  /*
    int it_size = ((size_cell -1) /2);
    
    for(int ny=y_cell-it_size; ny<=y_cell+it_size; ny++)
    {
      for(int nx=x_cell-it_size; nx<=x_cell+it_size; nx++)
      {
        grid->data[nx+ny*nCol] = 100;
      }
    }
    */

   grid->data[x_cell+y_cell*nCol] = 100;
  } 

  // call service to map_server
 // nav_msgs::SetMap map_update_srv;
  //map_update_srv.request.map = *grid;
  //set_map_client.call(map_update_srv);
  //std::cout << "Add Cubes Service Called" << std::endl;

  //publish modified map
  map_publisher.publish(*grid);

  return true;
}

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

void
loadMapFromFile(nav_msgs::GetMap::Response* resp,
                const char* fname, double res, bool negate,
                double occ_th, double free_th, double* origin,
                MapMode mode)
{
  SDL_Surface* img;

  unsigned char* pixels;
  unsigned char* p;
  unsigned char value;
  int rowstride, n_channels, avg_channels;
  unsigned int i,j;
  int k;
  double occ;
  int alpha;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if(!(img = IMG_Load(fname)))
  {
    std::string errmsg = std::string("failed to open image file \"") +
            std::string(fname) + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  resp->map.info.width = img->w;
  resp->map.info.height = img->h;
  resp->map.info.resolution = res;
  resp->map.info.origin.position.x = *(origin);
  resp->map.info.origin.position.y = *(origin+1);
  resp->map.info.origin.position.z = 0.0;
  btQuaternion q;
  // setEulerZYX(yaw, pitch, roll)
  q.setEulerZYX(*(origin+2), 0, 0);
  resp->map.info.origin.orientation.x = q.x();
  resp->map.info.origin.orientation.y = q.y();
  resp->map.info.origin.orientation.z = q.z();
  resp->map.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  resp->map.data.resize(resp->map.info.width * resp->map.info.height);

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  if (mode==TRINARY || !img->format->Amask)
    avg_channels = n_channels;
  else
    avg_channels = n_channels - 1;

  // Copy pixel data into the map structure
  pixels = (unsigned char*)(img->pixels);
  for(j = 0; j < resp->map.info.height; j++)
  {
    for (i = 0; i < resp->map.info.width; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j*rowstride + i*n_channels;
      color_sum = 0;
      for(k=0;k<avg_channels;k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)avg_channels;

      if (n_channels == 1)
          alpha = 1;
      else
          alpha = *(p+n_channels-1);

      if(negate)
        color_avg = 255 - color_avg;

      if(mode==RAW){
          value = color_avg;
          resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
          continue;
      }


      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied.  Otherwise, it's vice versa.
      occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if(occ > occ_th)
        value = +100;
      else if(occ < free_th)
        value = 0;
      else if(mode==TRINARY || alpha < 1.0)
        value = -1;
      else {
        double ratio = (occ - free_th) / (occ_th - free_th);
        value = 1 + 98 * ratio;
      }

      resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
    }
  }

  SDL_FreeSurface(img);
}

/** Load a map given all the values needed to understand it
 */
nav_msgs::OccupancyGrid::Ptr loadMapFromValues(std::string map_file_name, double resolution,
                        int negate, double occ_th, double free_th,
                        double origin[3], MapMode mode)
{
  nav_msgs::MapMetaData meta_data_message_;
  nav_msgs::GetMap::Response map_resp_;
  ROS_INFO("Loading map from image \"%s\"", map_file_name.c_str());
  try {
    loadMapFromFile(&map_resp_, map_file_name.c_str(),
                                resolution, negate, occ_th, free_th,
                                origin, mode);
  } catch (std::runtime_error& e) {
    ROS_ERROR("%s", e.what());
    return nullptr;
  }

  // To make sure get a consistent time in simulation
  ros::Time::waitForValid();
  map_resp_.map.info.map_load_time = ros::Time::now();
  map_resp_.map.header.frame_id = "map";
  map_resp_.map.header.stamp = ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
            map_resp_.map.info.width,
            map_resp_.map.info.height,
            map_resp_.map.info.resolution);
  meta_data_message_ = map_resp_.map.info;

  return boost::make_shared<nav_msgs::OccupancyGrid>(map_resp_.map);
}

bool Planner::load_map_yaml(std::string yaml_path)
{
  std::string mapfname;
  MapMode mode = TRINARY;
  double res;
  int negate;
  double occ_th;
  double free_th;
  double origin[3];
  std::ifstream fin(yaml_path.c_str());
  //if (fin.fail()) {
  //  ROS_ERROR("Map_server could not open %s.", yaml_path.c_str());
  //  return;
  //}

  // The document loading process changed in yaml-cpp 0.5.
  YAML::Node doc = YAML::LoadFile(yaml_path);

  try {
    //doc["resolution"] >> res;
    res = doc["resolution"].as<double>();
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
    return false;
  }
  try {
    //doc["negate"] >> negate;
    negate = doc["negate"].as<int>();
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain a negate tag or it is invalid.");
    return false;
  }
  try {
    //doc["occupied_thresh"] >> occ_th;
    occ_th = doc["occupied_thresh"].as<double>();
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
    return false;
  }
  try {
    //doc["free_thresh"] >> free_th;
    free_th = doc["free_thresh"].as<double>();
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
    return false;
  }
  try {
    std::string modeS = "";
    //doc["mode"] >> modeS;
    modeS = doc["mode"].as<std::string>();

    if(modeS=="trinary")
      mode = TRINARY;
    else if(modeS=="scale")
      mode = SCALE;
    else if(modeS=="raw")
      mode = RAW;
    else{
      ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
      return false;
    }
  } catch (YAML::Exception &) {
    ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
    mode = TRINARY;
  }
  try {
    //doc["origin"][0] >> origin[0];
    //doc["origin"][1] >> origin[1];
    //doc["origin"][2] >> origin[2];

    origin[0] = doc["origin"][0].as<double>();
    origin[1] = doc["origin"][1].as<double>();
    origin[2] = doc["origin"][2].as<double>();
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain an origin tag or it is invalid.");
    return false;
  }
  try {
    //doc["image"] >> mapfname;
    mapfname = doc["image"].as<std::string>();
    // TODO: make this path-handling more robust
    if(mapfname.size() == 0)
    {
      ROS_ERROR("The image tag cannot be an empty string.");
      return false;
    }

    boost::filesystem::path mapfpath(mapfname);
    if (!mapfpath.is_absolute())
    {
      boost::filesystem::path dir(yaml_path);
      dir = dir.parent_path();
      mapfpath = dir / mapfpath;
      mapfname = mapfpath.string();
    }
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain an image tag or it is invalid.");
    return false;
  }
  //update map
  setMap(loadMapFromValues(mapfname, res, negate, occ_th, free_th, origin, mode)); 

  //publish modified map
  map_publisher.publish(*grid);
  
  return true;
}
