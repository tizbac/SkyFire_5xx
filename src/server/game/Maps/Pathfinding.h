#ifndef PATHFINDING_H
#define PATHFINDING_H
#include <Common.h>
#include <Unit.h>
#include <Map.h>
#include <World.h>
#include "pathfinding/Detour/DetourNavMesh.h"
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <map>
#include <boost/graph/graph_concepts.hpp>
#define MAX_SMOOTH 400
#define STEP_SIZE 0.5f
extern uint32 maphashcount;
extern std::map<Map*,uint32> loadedmaps;
extern std::vector<boost::mutex *> unloadlocks;
extern boost::mutex loadedmapsmutex;
enum PathFindingSplineFlags
{
    PATHFINDING_SPLINEFLAG_WALKING        = 0x00001000,
    PATHFINDING_SPLINEFLAG_FLYING         = 0x00002000,
    PATHFINDING_SPLINEFLAG_CATMULL_ROM    = 0x00040000,
};
enum NavTerrain
{
    NAV_GROUND  = 0x01,

    NAV_MAGMA   = 0x02,
    NAV_SLIME   = 0x04,

    NAV_SHALLOW_WATER   = 0x08,
    NAV_AVERAGE_WATER   = 0x10,
    NAV_DEEP_WATER      = 0x20,
    NAV_SWIM_WATER      = 0x40,
    NAV_WATER           = NAV_SHALLOW_WATER | NAV_AVERAGE_WATER | NAV_DEEP_WATER | NAV_SWIM_WATER,

    NAV_UNSPECIFIED     = 0x80
};
enum pathfindErrorResult{
        PATHFIND_OK,
        PATHFIND_NO_PATHS_TO_TARGET,
        PATHFIND_CANT_FIND_START_NAVMESH,
        PATHFIND_CANT_FIND_END_NAVMESH,
        PATHFIND_ERROR_NAVMESH,
        PATHFIND_NAVMESH_NOT_LOADED
};
enum PFstate{
  PATHFINDINGSTATUS_DEST_UNREACHABLE = 0x01,
  PATHFINDINGSTATUS_PARTIAL = 0x02,
  PATHFINDINGSTATUS_FINAL = 0x03,
  PATHFINDINGSTATUS_BROKENNAVMESH =0x04
};
struct pathfindResult{
        float * path;
        pathfindErrorResult result;
        uint32 waypointcount;
};

enum PathfindingStatus{
  PATHFIND_DESTINATION_REACHED = 0,
  PATHFIND_DESTINATION_UNREACHABLE = 1
};
class TrinityVector3{
  public:
  float x,y,z;
  TrinityVector3()
  {
    x = 0;
    y = 0;
    z = 0;
  }
  TrinityVector3(float x,float y ,float z)
  {
    this->x = x;this->y = y;this->z = z;
  }
  TrinityVector3 operator+(TrinityVector3 b)
  {
    return TrinityVector3(x+b.x,y+b.y,z+b.z);
  }
  TrinityVector3 operator-(TrinityVector3 b)
  {
    return TrinityVector3(x-b.x,y-b.y,z-b.z);
  }
  TrinityVector3 operator*(float b)
  {
    return TrinityVector3(x*b,y*b,z*b);
  }
  TrinityVector3 operator/(float b)
  {
      return TrinityVector3(x/b,y/b,z/b);

  }
  float Dot(TrinityVector3& b)
  {
      return x*b.x+y*b.y+z*b.z;

  }

  TrinityVector3 operator=(TrinityVector3 b)
  {
    x = b.x;
    y = b.y;
    z = b.z;
    return TrinityVector3(x,y,z);
  }
  float dist(TrinityVector3 b)
  {
    return sqrt((x-b.x)*(x-b.x)+(y-b.y)*(y-b.y)+(z-b.z)*(z-b.z));


  }
  float mag()
  {
    return sqrt((x)*(x)+(y)*(y)+(z)*(z));
  }
  TrinityVector3 normalize()
  {
    if ( mag() > 0.0 )
      return TrinityVector3(x/mag(),y/mag(),z/mag());
    return TrinityVector3(0,0,0);
  }
  bool operator==(TrinityVector3 b)
  {
    return x==b.x&&y==b.y&&z==b.z;
  }
  std::string as_str()
  {
    std::stringstream ss;
    ss << "TrinityVector3(" << x << "," << y << "," << z << ") ";
    return ss.str();
    
  }


};
class PathFindingMgr;
class PathViewer;
class PathFindingState
{
  public:
    bool calculated;
    uint32 unittypeid;
    std::vector<float> path;
    uint32 wpcount;
    uint32 currtime;
    uint32 currwp;
    uint32 origwpcount;
    uint32 lastposupdate;
    uint32 id;
    uint64 guid;
    std::vector<uint32> waypointtime;
    boost::asio::io_service arrivedcallback;
    Map * map;
    void * destholdptr;
    float destx;
    float desty;
    float destz;
    float lastx;
    float lasty;
    float lastz;
    TrinityVector3 petownerposition;
    bool arrived;
    bool pause;
    PathViewer * debug;
    float speed;
        uint32 targetmflags;
    uint32 status;
    bool mustrecalculate;
    void Calculate(Map * m,Unit * u);
    void Pause();
    void UnPause();
    void UpdateDestination(float destx,float desty,float destz,uint32 movementflags);
    bool HasArrived();
    bool willdelete;
    PathFindingMgr * pmgr;
    TrinityVector3 GetPositionNow();
    boost::mutex statelock;
    ~PathFindingState();
    
    
    bool isCharge;

};
typedef std::list<uint64> t_mobwplist;
class PathViewer
{
  
public:
    PathViewer(Unit * u);
    ~PathViewer();
    void UpdatePath(std::vector<float> path);
private:
     t_mobwplist waypointmobs;
     uint64 ownerguid;
};
class PathFindingMgr
{
  public:
    std::list<PathFindingState*> paths;
    std::list<PathFindingState*> removelist;
    std::list<PathFindingState*> addlist;
    boost::mutex listsmutex;
    boost::mutex statedeletelock;
    bool exit;
    Map * map;

    uint32 currdiff;
    uint32 pathfindingcount;
    PathFindingState* AddPathfind(Unit * u,float destx, float desty, float destz,float speed,bool isCharge = false);
    void SendMonsterMoveGUID(uint64 guid,std::vector<float> path,uint32 time,uint32 timestamp);
    void PlayerRelocateGUID(uint64 guid,float x , float y,float z);
    void CreatureRelocateGUID(uint64 guid,float x , float y,float z);
    
    //PathFindingState * GetPathfindState(Unit * u);
    void RemovePathfind(PathFindingState * pf);
    bool IsValid(PathFindingState * pf);
};


#endif // NO_H
