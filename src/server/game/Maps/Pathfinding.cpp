#include "Pathfinding.h"
#include "pathfinding/InputGeom.h"
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "ObjectAccessor.h"
#include <math.h>
#include <valarray>
#include <sys/times.h>
#include <float.h>
#include "Player.h"
#include "Creature.h"
#include "TemporarySummon.h"

bool isUnitOnGround(Unit * u);

uint32 maphashcount;
std::map<Map*,uint32> loadedmaps;
std::vector<boost::mutex *> unloadlocks;// Chiave: allocId % numthreads , Valore: mutex del gruppo di mappe
std::map<PathViewer*,int> validpv;

inline uint32 packTileID(uint32 tileX, uint32 tileY) {
    return tileX<<16 | tileY;
}
inline void unpackTileID(uint32 ID, uint32 &tileX, uint32 &tileY) {
    tileX = ID>>16;
    tileY = ID&0xFF;
}
void Map::LoadNavMesh(int gx, int gy)
{
    boost::mutex::scoped_lock lock(navmeshmutex);
    int i_id = GetId();
    char fileName[512];
    FILE* file;
    long bytesread;
    if(!m_navMesh)
    {
        sprintf(fileName, "%smmaps/%03i.mmap", sWorld->GetDataPath().c_str(), i_id);
        file = fopen(fileName, "rb");

        if(!file)
        {

            sprintf(fileName, "%smmaps/%03i_1.mmap", sWorld->GetDataPath().c_str(), i_id);
            file = fopen(fileName, "rb");
            if (!file)
            {
              TC_LOG_DEBUG("maps", "Error: Could not open mmap file '%s'", fileName);
              return;
            }
        }

        dtNavMeshParams params;
        bytesread = fread(&params, sizeof(dtNavMeshParams), 1, file);
        if ( bytesread < 1 )
        {
            TC_LOG_ERROR("general", "%d:Short read while loading '%s' got %u bytes, expected %u",__LINE__,fileName,uint32(bytesread*sizeof(dtNavMeshParams)), uint32(sizeof(dtNavMeshParams)));
            fclose(file);
            return;
            
        }
        /*if ( params.maxTiles < 65535 )
        {
          sLog->outError("Attenzione: Ci sono troppi pochi tile sulla navmesh %03u.mmap, Ridimensionamento",i_id);
          params.maxTiles = 65535;
          
        }*/
        fclose(file);

        m_navMesh = new dtNavMesh;
        if(!m_navMesh->init(&params))
        {
            delete m_navMesh;
            m_navMesh = 0;
            TC_LOG_ERROR("general", "Error: Failed to initialize mmap %03u from file %s", i_id, fileName);
            return;
        }
    }

    uint32 packedGridPos = packTileID(uint32(gx), uint32(gy));
    if(m_mmapTileMap.find(packedGridPos) != m_mmapTileMap.end())
        return;

    // mmaps/0000000.mmtile
    sprintf(fileName, "%smmaps/%03i%02i%02i.mmtile", sWorld->GetDataPath().c_str(), i_id, gx, gy);
    file = fopen(fileName, "rb");

    if(!file)
    {
        sprintf(fileName, "%smmaps/%03i%02i%02i_1.mmtile", sWorld->GetDataPath().c_str(), i_id, gx, gy);
        file = fopen(fileName, "rb");
        if (!file)
        {
          TC_LOG_DEBUG("maps", "Error: Could not open mmtile file '%s'", fileName);
          return;
        }
    }

    fseek(file, 0, SEEK_END);
    int length = ftell(file);
    fseek(file, 0, SEEK_SET);

    unsigned char* data =  (unsigned char*)dtAlloc(length, DT_ALLOC_PERM);
    bytesread = fread(data, length, 1, file);
    if ( bytesread < 1 )
    {
        TC_LOG_ERROR("general", "%d:Short read while loading '%s' got %d bytes, expected %d",__LINE__,fileName,int32(bytesread*length), length);
        return;
    }
    fclose(file);

    dtMeshHeader* header = (dtMeshHeader*)data;
    if (header->magic != DT_NAVMESH_MAGIC)
    {
        TC_LOG_ERROR("general", "Error: %03u%02i%02i.mmtile has an invalid header", i_id, gx, gy);
        dtFree(data);
        return;
    }
    if (header->version != DT_NAVMESH_VERSION)
    {
        TC_LOG_ERROR("general", "Error: %03u%02i%02i.mmtile was built with Detour v%i, expected v%i",
                              i_id, gx, gy,                 header->version, DT_NAVMESH_VERSION);
        dtFree(data);
        return;
    }

    if(!m_navMesh->addTile(data, length, DT_TILE_FREE_DATA,0,NULL))
    {
        TC_LOG_ERROR("general", "Error: could not load %03u%02i%02i.mmtile into navmesh (InstanceId: %d)", i_id, gx, gy,GetInstanceId());
        dtFree(data);
        return;
    }

    // memory allocated for data is now managed by detour, and will be deallocated when the tile is removed

    uint32 packedTilePos = packTileID(uint32(header->x), uint32(header->y));
    m_mmapTileMap.insert(std::pair<uint32, uint32>(packedGridPos, packedTilePos));
    TC_LOG_DEBUG("maps", "Loaded mmtile %03i[%02i,%02i] into %03i(%u)[%02i,%02i]", i_id, gx, gy, i_id, GetInstanceId(), header->x, header->y);
}
bool Map::NavMeshLoaded(int gx, int gy)
{
   boost::mutex::scoped_lock lock(navmeshmutex);
   uint32 packedGridPos = packTileID(uint32(gx), uint32(gy));
    if(m_mmapTileMap.find(packedGridPos) != m_mmapTileMap.end())
        return true; else return false;

}


/*namespace boost
{

void assertion_failed(char const * expr, char const * function, char const * file, long line)
  {
    std::cerr << "ASSERZIONE '" << expr << "' FALLITA! " << function << " " << file << " " << line << std::endl;
    abort();
  }
} */

#ifndef isnan
#define isnan(x) std::isnan(x)
#endif
#ifndef isinf
#define isinf(x) std::isinf(x)
#endif



float Interpolate(float x1,float x2,float frac)
{
    float rx = x1+(x2-x1)*frac;

    return rx;
}
inline double IntersectSphere_t0(double sx,double sy,double sz,double DX,double DY,double DZ, double cx, double cy,double cz, double r)
{
    
    double delta = -(DX*DX)*(cy*cy)-(DY*DY)*(cx*cx)-(DX*DX)*(cz*cz)
    -(DZ*DZ)*(cx*cx)-(DY*DY)*(cz*cz)-(DZ*DZ)*(cy*cy)+(DX*DX)*(r*r)+(DY*DY)*(r*r)+(DZ*DZ)*(r*r)-
    (DX*DX)*(sy*sy)-(DY*DY)*(sx*sx)-(DX*DX)*(sz*sz)-(DZ*DZ)*(sx*sx)-(DY*DY)*(sz*sz)-(DZ*DZ)*(sy*sy)+
    (DY*DY)*cx*sx*2.0+(DX*DX)*cy*sy*2.0+(DZ*DZ)*cx*sx*2.0+(DX*DX)*cz*sz*2.0+(DZ*DZ)*cy*sy*2.0+
    (DY*DY)*cz*sz*2.0+DX*DY*cx*cy*2.0+DX*DZ*cx*cz*2.0+DY*DZ*cy*cz*2.0-DX*DY*cx*sy*2.0-
    DX*DY*cy*sx*2.0-DX*DZ*cx*sz*2.0-DX*DZ*cz*sx*2.0-DY*DZ*cy*sz*2.0-DY*DZ*cz*sy*2.0+DX*DY*sx*sy*2.0+
    DX*DZ*sx*sz*2.0+DY*DZ*sy*sz*2.0;
    if ( delta < 0.0f )
        return G3D::nan();
    double t0 = -(-DX*cx-DY*cy-DZ*cz+DX*sx+DY*sy+DZ*sz+sqrt(delta))/(DX*DX+DY*DY+DZ*DZ);
    return t0;
}
inline double IntersectSphere_t1(double sx,double sy,double sz,double DX,double DY,double DZ, double cx, double cy,double cz, double r)
{
    double delta = -(DX*DX)*(cy*cy)-(DY*DY)*(cx*cx)-(DX*DX)*(cz*cz)-(DZ*DZ)*(cx*cx)
    -(DY*DY)*(cz*cz)-(DZ*DZ)*(cy*cy)+(DX*DX)*(r*r)+(DY*DY)*(r*r)+(DZ*DZ)*(r*r)-(DX*DX)*(sy*sy)-(DY*DY)*(sx*sx)
    -(DX*DX)*(sz*sz)-(DZ*DZ)*(sx*sx)-(DY*DY)*(sz*sz)-(DZ*DZ)*(sy*sy)+(DY*DY)*cx*sx*2.0+(DX*DX)*cy*sy*2.0+(DZ*DZ)*cx*sx*2.0
    +(DX*DX)*cz*sz*2.0+(DZ*DZ)*cy*sy*2.0+(DY*DY)*cz*sz*2.0+DX*DY*cx*cy*2.0+DX*DZ*cx*cz*2.0+DY*DZ*cy*cz*2.0-DX*DY*cx*sy*2.0
    -DX*DY*cy*sx*2.0-DX*DZ*cx*sz*2.0-DX*DZ*cz*sx*2.0-DY*DZ*cy*sz*2.0-DY*DZ*cz*sy*2.0+DX*DY*sx*sy*2.0+DX*DZ*sx*sz*2.0+DY*DZ*sy*sz*2.0;
    if ( delta < 0.0f )
        return G3D::nan();
    double t1 = (DX*cx+DY*cy+DZ*cz-DX*sx-DY*sy-DZ*sz+sqrt(delta))/(DX*DX+DY*DY+DZ*DZ);
    return t1;
}
bool NewContactPoint( TrinityVector3<double> target , float radius , std::vector<float>& path , TrinityVector3<double>* last_point)
{
    TrinityVector3<double> startpoint;
    TrinityVector3<double> endpoint;
    TrinityVector3<double> direction;
    TrinityVector3<double> intersection;
    
    TrinityVector3<double> reference;
//     std::stringstream ss;
    
    for ( int i = 0; i < path.size()/3-1; i++ )
    {
        startpoint.x = path[i*3+0];startpoint.y = path[i*3+1];startpoint.z = path[i*3+2];
        if ( i == 0 )
        {
            reference = startpoint;
//             ss << "plot(plot::Circle2d(" << radius << ",[" << (target-reference).x << "," << (target-reference).y << "]),"; 
        }
        endpoint.x = path[(i+1)*3+0]; endpoint.y = path[(i+1)*3+1]; endpoint.z = path[(i+1)*3+2];
        
        direction = endpoint-startpoint; // t andrà da 0 a 1
//         ss << "plot::Curve2d([" << (startpoint-reference).x << "+" << direction.x << "*t," << (startpoint-reference).y << "+" << direction.y << "*t],t=0..1),";
        
        float t0 = IntersectSphere_t0(startpoint.x,startpoint.y,startpoint.z,
                                      direction.x,direction.y,direction.z,
                                      target.x,target.y,target.z,radius
        );
        float t1 = IntersectSphere_t1(startpoint.x,startpoint.y,startpoint.z,
                                      direction.x,direction.y,direction.z,
                                      target.x,target.y,target.z,radius
        );
        if ( !isnan(t0) && t0 >= 0.0f && t0 <= 1.0f )
        {
            intersection = startpoint+direction*t0;
//             printf("Intersezione(target = %s ) t0 da %s a %s , collisione in %s con sfera di raggio %f , terminazione del percorso\n",target.as_str().c_str(),startpoint.as_str().c_str(),
//                            endpoint.as_str().c_str(),intersection.as_str().c_str(),radius
//             );
            //Sostituisce il prossimo waypoint con intersection
            path[(i+1)*3+0] = intersection.x;
            path[(i+1)*3+1] = intersection.y;
            path[(i+1)*3+2] = intersection.z;
            //Tronca il percorso
            path.resize((i+1)*3+3);
            *last_point = intersection;
//             ss << "[" << (intersection-reference).x << "," << (intersection-reference).y << "," << (intersection-reference).z << "])";
//             std::cout << ss.str() << std::endl;
            return true;
        }
        if ( !isnan(t1) && t1 >= 0.0f && t1 <= 1.0f )
        {
            intersection = startpoint+direction*t1;
//             printf("Intersezione(target = %s ) t1 da %s a %s , collisione in %s con sfera di raggio %f , terminazione del percorso\n",target.as_str().c_str(),startpoint.as_str().c_str(),
//                            endpoint.as_str().c_str(),intersection.as_str().c_str(),radius
//             );
            //Sostituisce il prossimo waypoint con intersection
            path[(i+1)*3+0] = intersection.x;
            path[(i+1)*3+1] = intersection.y;
            path[(i+1)*3+2] = intersection.z;
            //Tronca il percorso
            path.resize((i+1)*3+3);
            *last_point = intersection;
//             ss << "[" << (intersection-reference).x << "," << (intersection-reference).y << "," << (intersection-reference).z << "])";
//             std::cout << ss.str() << std::endl;
            return true;
        }
//         printf("Nessuna Intersezione(target = %s ) t1 da %s a %s , con sfera di raggio %f , terminazione del percorso\n",target.as_str().c_str(),startpoint.as_str().c_str(),
//                            endpoint.as_str().c_str(),radius
//             );
    }
//     ss << ")";
//     std::cout << ss.str() << std::endl;
    return false;
    
}


///Queste funzioni sono utilizzate sul thread del world per eseguire le azioni messe in coda dal pathfinding
void PathFindingMgr::SendMonsterMoveGUID(uint64 guid,std::vector< float > path, float speed,uint32 id, uint64 facingTarget)
{
    Unit * u;
    u = ObjectAccessor::GetObjectInWorld(guid,(Unit*)NULL);
    if (!u)
    {
        TC_LOG_ERROR("general", "SendMonsterMoveGUID: GUID " UI64FMTD " non valida.",guid);

        return;
    }
    //sLog->outError("SendMonsterMoveGUID");
    bool fly;
    fly = ( u->GetUnitMovementFlags() & MOVEMENTFLAG_DISABLE_GRAVITY ) != 0;
    u->SendMonsterMove(path,speed,id,true,fly, facingTarget);

}
void PathFindingMgr::CreatureRelocateGUID(uint64 guid, float x, float y, float z)
{
    Unit * c;
    c = ObjectAccessor::GetObjectInWorld(guid,(Unit*)NULL);
    if (!c)
    {
        TC_LOG_ERROR("general", "CreatureRelocateGUID: GUID " UI64FMTD " non valida.",guid);
        return;
    }
    c->UpdatePosition(x,y,z,c->GetOrientation());
    //c->Relocate(x,y,z);
}
void PathFindingMgr::PlayerRelocateGUID(uint64 guid, float x, float y, float z)
{
    Player* p;
    p = ObjectAccessor::GetObjectInWorld(guid,(Player*)NULL);
    if (!p)
        return;
    p->UpdatePosition(x,y,z,p->GetOrientation());
    //p->Relocate(x,y,z);
}
void PathFindingDebug_DelayedDelete(PathViewer * v)
{
    delete v;
}
void UnitStopMoving(uint64 guid)
{
    Player * p;
    p = ObjectAccessor::GetObjectInWorld(guid,(Player*)NULL);
    if ( p )
    {
        p->StopMoving();

    } else {
        Unit * u;
        u = ObjectAccessor::GetObjectInWorld(guid,(Unit*)NULL);
        if (!u)
        {
            TC_LOG_ERROR("general", "UnitStopMoving: GUID " UI64FMTD " non valida.",guid);
        }
        u->StopMoving();

    }
}
void UnitRelocateGUID(uint64 guid, float x, float y, float z)
{

    Player * p;
    p = ObjectAccessor::GetObjectInWorld(guid,(Player*)NULL);
    if (!p)
    {
        Unit * c;
        c = ObjectAccessor::GetObjectInWorld(guid,(Unit*)NULL);
        if (!c)
        {
            TC_LOG_ERROR("general", "CreatureRelocateGUID: GUID " UI64FMTD " non valida.",guid);
            return;
        }
        c->UpdatePosition(x,y,z,c->GetOrientation());
    } else {
        p->UpdatePosition(x,y,z,p->GetOrientation());
    }
    //c->Relocate(x,y,z);
}

///--------------




void UpdateMapPathfinding(Map * m,uint32 timediff)
{
    PathFindingMgr * pMgr = m->GetPathFindingMgr();

    //pMgr->exit = false;
    boost::system_time st;
// while ( not pMgr->exit )
// {

    {   // Rimuove il pathfinding dai movement generator deletati

        boost::mutex::scoped_lock lock(pMgr->listsmutex);
        for ( std::list<PathFindingState*>::iterator it = pMgr->removelist.begin(); it != pMgr->removelist.end(); it++ )
        {
            for ( std::list<PathFindingState*>::iterator it2 = pMgr->paths.begin(); it2 != pMgr->paths.end(); it2++ )
            {

                if ( ((PathFindingState*)(*it2)) == ((PathFindingState*)(*it)) )
                {
                    /*Unit * unitptr = ObjectAccessor::GetObjectInMap(*it,m,(Unit*)NULL;
                    if ( unitptr )
                        m->mtcalls.post(boost::bind(&Unit::StopMoving,unitptr));*/
                    boost::mutex::scoped_lock lock2(pMgr->statedeletelock);
                    //printf("\033[31mREMOVING GUID %llu\033[0m",(*it)->guid);
                    PathFindingState* sta = *it;

                    pMgr->paths.erase(it2);
                    if ( sta->debug )
                        m->mtcalls->post(boost::bind(&PathFindingDebug_DelayedDelete,sta->debug));
                    delete sta;
                    break;

                }
            }
        }
        for ( std::list<PathFindingState*>::iterator it = pMgr->addlist.begin(); it != pMgr->addlist.end(); it++ )
        {
            //printf("\033[31mADDING GUID %llu\033[0m",(*it)->guid);

            pMgr->paths.push_back(*it);
        }
        pMgr->addlist.clear();


        pMgr->removelist.clear();







    }



    //TC_LOG_DEBUG("maps", "MovementThread: Update diff = %d, pathfinderscount=%d",timediff,pMgr->paths.size());
    pMgr->pathfindingcount = pMgr->paths.size();
    for ( std::list<PathFindingState*>::iterator it = pMgr->paths.begin(); it != pMgr->paths.end(); it++ )
    {

        //boost::mutex::scoped_lock crlock(sObjectAccessor->creaturelock);

        PathFindingState * pfstate = *it;
        if ( pfstate->willdelete )
        {
            pMgr->removelist.push_back(pfstate);
            continue;

        }
        boost::mutex::scoped_lock lock2(pfstate->statelock);
        sObjectAccessor->deletelock.lock();//Impedisci di rimuovere player o creature durante l'update o aspetta la conclusione di un eventuale logout
        Unit * u = ObjectAccessor::GetObjectInMap<Unit>(pfstate->guid,m,(Unit*)NULL);//Ottiene la Unit dalla guid
        if (! u )//Dato che non esiste più può essere rimosso
        {
            boost::mutex::scoped_lock lock(pMgr->listsmutex);
            pMgr->removelist.push_back(pfstate);
            TC_LOG_DEBUG("maps", "La Unit è stata deletata prima di deletare il movementgenerator");
            sObjectAccessor->deletelock.unlock();
            continue;
        }
        if ( u->GetMap() != m)//non dovrebbe mai succedere in teoria
        {
            TC_LOG_ERROR("general", "Unit '%s' guid " UI64FMTD " : Pathfindingthread: La mappa della Unit non corrisponde al thread , rimozione ",u->GetName().c_str(),u->GetGUID());
            boost::mutex::scoped_lock lock(pMgr->listsmutex);
            pMgr->removelist.push_back(pfstate);
            sObjectAccessor->deletelock.unlock();
            continue;

        }
        boost::mutex::scoped_lock dllock(u->deletelock);
        sObjectAccessor->deletelock.unlock();
        
        if ( pfstate->unittypeid == TYPEID_UNIT )
        {
            if ( GetMSTimeDiffToNow(pfstate->lastCheck) > 200)
            {
                bool fly = !isUnitOnGround(u);
                //printf("%s flying: %s\n",u->GetName(),fly ? "yes" : "no");
                Creature * c = u->ToCreature();
                if ( c && c->GetCreatureTemplate()->InhabitType & INHABIT_AIR)
                {
                    if ( fly && !pfstate->isflying )
                    {
                        u->AddUnitMovementFlag(MOVEMENTFLAG_FLYING | MOVEMENTFLAG_DISABLE_GRAVITY | MOVEMENTFLAG_CAN_FLY );
                    }
                    if ( !fly && pfstate->isflying )
                    {
                        u->RemoveUnitMovementFlag(MOVEMENTFLAG_FLYING | MOVEMENTFLAG_DISABLE_GRAVITY | MOVEMENTFLAG_CAN_FLY );
                    }
                }else{//I mob non volanti non possono volare
                    u->RemoveUnitMovementFlag(MOVEMENTFLAG_FLYING | MOVEMENTFLAG_DISABLE_GRAVITY | MOVEMENTFLAG_CAN_FLY );
                }
                pfstate->isflying = fly;
                pfstate->lastCheck = getMSTime();
                
                
                //CHECK LOS
                if ( !isnan( pfstate->chasetargetposition.x) && pfstate->GetPositionNow().dist(pfstate->chasetargetposition) < (u->GetMeleeReach()+pfstate->targetmeleerange)*1.1 )
                {
                    TrinityVector3<float> coll_point;
                    
                    bool isobs = !m->NavMeshLOS(pfstate->chasetargetposition.x,pfstate->chasetargetposition.y,pfstate->chasetargetposition.z,pfstate->destx,pfstate->desty,pfstate->destz,&coll_point);
                    if ( isobs )
                    {
                        pfstate->UpdateDestination(coll_point.x,coll_point.y,coll_point.z,u->GetUnitMovementFlags(),false);
                    }
                }
                
            }
        }

        if ( (! pfstate->calculated) || ( pfstate->mustrecalculate ) )
        {
            TC_LOG_DEBUG("maps", "<%s>Percorso non calcolato oppure ricalcolazione necessaria, calcolo...",u->GetName().c_str());
            pfstate->Calculate(m,u);
            if ( pfstate->path.size() > 3 && !pfstate->pause && pfstate->status != PATHFINDINGSTATUS_DEST_UNREACHABLE )//Invia il percorso solo se valido
                m->mtcalls->post(boost::bind(&PathFindingMgr::SendMonsterMoveGUID,pMgr,u->GetGUID(),pfstate->path2send,pfstate->speed,pfstate->id, pfstate->facingTarget));
            else
                TC_LOG_DEBUG("maps", "<%s>Percorso calcolato non valido %u %d %u",u->GetName().c_str(),uint32(pfstate->path.size()),pfstate->pause,pfstate->status);
            pfstate->mustrecalculate = false;
        }
        if ( pfstate->pause || pfstate->arrived || pfstate->status == PATHFINDINGSTATUS_DEST_UNREACHABLE )
        {
            /*if ( ! sqrt((pfstate->lastx-pfstate->destx)*(pfstate->lastx-pfstate->destx)+(pfstate->lasty-pfstate->desty)*(pfstate->lasty-pfstate->desty)+(pfstate->lastz-pfstate->destz)*(pfstate->lastz-pfstate->destz)) < 0.03 )
              pfstate->arrived = false;*/
           // TC_LOG_DEBUG("maps", "<%s> IN PAUSA O ARRIVATO",u->GetName());
            continue;
        }

        if ( pfstate->currwp >= pfstate->wpcount-1 )
        {
            if ( pfstate->status == PATHFINDINGSTATUS_FINAL )
            {
                TC_LOG_DEBUG("maps", "<%s> Arrivato (%f %f %f).",u->GetName().c_str(),pfstate->lastx,pfstate->lasty,pfstate->lastz);

                pfstate->arrived = true;
                pfstate->lastx = pfstate->destx;
                pfstate->lasty = pfstate->desty;
                pfstate->lastz = pfstate->destz;
                if ( u->GetTypeId() == TYPEID_PLAYER )
                    m->mtcalls->post(boost::bind(&PathFindingMgr::PlayerRelocateGUID,pMgr,u->GetGUID(),pfstate->destx,pfstate->desty,pfstate->destz));
                else
                    m->mtcalls->post(boost::bind(&PathFindingMgr::CreatureRelocateGUID,pMgr,u->GetGUID(),pfstate->destx,pfstate->desty,pfstate->destz));

                continue;
            } else if ( pfstate->status == PATHFINDINGSTATUS_PARTIAL ) {
                TC_LOG_DEBUG("maps", "<%s> Calcolo nuova parte di percorso",u->GetName().c_str());

                pfstate->Calculate(m,u);
                if ( pfstate->path.size() > 3 )
                    m->mtcalls->post(boost::bind(&PathFindingMgr::SendMonsterMoveGUID,pMgr,u->GetGUID(),pfstate->path2send,pfstate->speed,pfstate->id, pfstate->facingTarget));

            }
            continue;


        }
        if ( pfstate->currwp >= pfstate->wpcount-1 )
            continue;
        if ( pfstate->status == PATHFINDINGSTATUS_DEST_UNREACHABLE )
            continue;
        {


            if ( pfstate->unittypeid == TYPEID_PLAYER )//Aggiorna la posizione
            {
                Player * p = (Player*)u;
                //TC_LOG_DEBUG("maps", "CurrTime: %d waypointtime[%d] = %d",pfstate->currtime,pfstate->currwp,pfstate->waypointtime[pfstate->currwp]);
                if ( pfstate->currtime >= pfstate->waypointtime[pfstate->currwp+1])
                {
                    //TC_LOG_DEBUG("maps", "Waypoint %d , diff = %d",pfstate->currwp,timediff);
                    /*float x = pfstate->lastx+(pfstate->path[pfstate->currwp*3+0]-pfstate->lastx)*coeff;//Crea un movimento + lungo così che non si formano scatti in caso di lag
                    float y = pfstate->lasty+(pfstate->path[pfstate->currwp*3+1]-pfstate->lasty)*coeff;// In questo modo quando si cambia waypoint è molto difficile che la unit sia già arrivata nella posizione
                    float z = pfstate->lastz+(pfstate->path[pfstate->currwp*3+2]-pfstate->lastz)*coeff;*/

                    //traveller.MoveTo(pfstate->path[pfstate->currwp*3+0],pfstate->path[pfstate->currwp*3+1],pfstate->path[pfstate->currwp*3+2],traveller.GetTotalTrevelTimeTo(pfstate->path[pfstate->currwp*3+0],pfstate->path[pfstate->currwp*3+1],pfstate->path[pfstate->currwp*3+2]));
                    /* if ( pfstate->currtime - pfstate->lastposupdate > 100)
                     {
                         m->mtcalls.post(boost::bind(&PathFindingMgr::PlayerRelocateGUID,pMgr,p->GetGUID(),pfstate->path[(pfstate->currwp)*3+0],pfstate->path[(pfstate->currwp)*3+1],pfstate->path[(pfstate->currwp)*3+2]));
                         pfstate->lastposupdate = pfstate->currtime;
                     }

                     pfstate->lastx = pfstate->path[(pfstate->currwp)*3+0];
                     pfstate->lasty = pfstate->path[(pfstate->currwp)*3+1];
                     pfstate->lastz = pfstate->path[(pfstate->currwp)*3+2];
                     */pfstate->currwp++;
                }
                // }else{
// //
                if ( pfstate->currtime - pfstate->lastposupdate > 100)
                {
                    TrinityVector3<float> currpos = pfstate->GetPositionNow();
                    m->mtcalls->post(boost::bind(&PathFindingMgr::PlayerRelocateGUID,pMgr,p->GetGUID(),currpos.x,currpos.y,currpos.z));
                    pfstate->lastposupdate = pfstate->currtime;
                    pfstate->lastx = currpos.x;
                    pfstate->lasty = currpos.y;
                    pfstate->lastz = currpos.z;
                }





                // }
            }
            if ( pfstate->unittypeid == TYPEID_UNIT )
            {
                
                //TC_LOG_DEBUG("maps", "CurrTime: %d waypointtime[%d] = %d, arrived = %s",pfstate->currtime,pfstate->currwp,pfstate->waypointtime[pfstate->currwp],pfstate->arrived ? "true" : "false");
                if ( pfstate->currtime >= pfstate->waypointtime[pfstate->currwp+1] )
                {
                    //
                    /*float x = pfstate->lastx+(pfstate->path[pfstate->currwp*3+0]-pfstate->lastx)*coeff;//Crea un movimento + lungo così che non si formano scatti in caso di lag
                    float y = pfstate->lasty+(pfstate->path[pfstate->currwp*3+1]-pfstate->lasty)*coeff;// In questo modo quando si cambia waypoint è molto difficile che la unit sia già arrivata nella posizione
                    float z = pfstate->lastz+(pfstate->path[pfstate->currwp*3+2]-pfstate->lastz)*coeff;*/

                    //traveller.MoveTo(pfstate->path[pfstate->currwp*3+0],pfstate->path[pfstate->currwp*3+1],pfstate->path[pfstate->currwp*3+2],traveller.GetTotalTrevelTimeTo(pfstate->path[pfstate->currwp*3+0],pfstate->path[pfstate->currwp*3+1],pfstate->path[pfstate->currwp*3+2]));
                    /*if ( pfstate->currtime - pfstate->lastposupdate > 100)
                    {
                        m->mtcalls.post(boost::bind(&PathFindingMgr::CreatureRelocateGUID,pMgr,u->GetGUID(),pfstate->path[(pfstate->currwp)*3+0],pfstate->path[(pfstate->currwp)*3+1],pfstate->path[(pfstate->currwp)*3+2]));
                        pfstate->lastposupdate = pfstate->currtime;
                    }
                    pfstate->lastx = pfstate->path[(pfstate->currwp)*3+0];
                    pfstate->lasty = pfstate->path[(pfstate->currwp)*3+1];
                    pfstate->lastz = pfstate->path[(pfstate->currwp)*3+2];*/
                    pfstate->currwp++;
                }//else{

                if ( pfstate->currtime - pfstate->lastposupdate > 100)
                {
                    TrinityVector3<float> currpos = pfstate->GetPositionNow();
                    m->mtcalls->post(boost::bind(&PathFindingMgr::CreatureRelocateGUID,pMgr,u->GetGUID(),currpos.x,currpos.y,currpos.z));
                    pfstate->lastposupdate = pfstate->currtime;
                    pfstate->lastx = currpos.x;
                    pfstate->lasty = currpos.y;
                    pfstate->lastz = currpos.z;
                }



                //}
            }

            pfstate->currtime += timediff;


        }


    }







    //boost::thread::sleep(sleeptime);
    //boost::posix_time::time_duration dif = boost::get_system_time()-st;
    //timediff = dif.total_milliseconds();
    // *timediffptr += dif.total_milliseconds();
// }





}
/*!
Blocca il movimento della Unit posizionandola sul punto corrente
*/
void PathFindingState::Pause()//Queste tre funziono vnano chiamate solo dal thread del world
{

    Unit * u = ObjectAccessor::GetObjectInWorld(guid,(Unit*)NULL);

    if ( u )
    {
        TC_LOG_DEBUG("maps", "Movement %p: Pause()",this);
        Map * m = u->GetMap();

        u->StopMoving();
        statelock.lock();
        if ( calculated )
        {
            TrinityVector3<float> pos = GetPositionNow();


            Map * m = u->GetMap();
            if ( m )
            {
                m->mtcalls->post(boost::bind(UnitRelocateGUID,u->GetGUID(),pos.x,pos.y,pos.z)); //Se chiamata direttamente Unit::SetPosition può causare deadlock in quanto
                //Togliendosi una spell che causa charm con il movimento , la unit non più charmata può far partire un altro movimento senza che sia stato rilasciato il lock dal movement generator precedente
                //u->SetPosition(pos.x,pos.y,pos.z,u->GetOrientation());
            }

            lastx = pos.x;
            lasty = pos.y;
            lastz = pos.z;
        }

        statelock.unlock();
    }
    statelock.lock();
    pause = true;
    statelock.unlock();
}
/*!
Consente di sapere se la Unit è arrivata alla fine del percorso
*/
bool PathFindingState::HasArrived()
{
    boost::mutex::scoped_lock lock(statelock);
    Unit * u = ObjectAccessor::GetObjectInWorld(guid,(Unit*)NULL);
    if ( (!calculated) || (mustrecalculate) )
        return false;
    if ( status == PATHFINDINGSTATUS_PARTIAL )
        return false;
    if ( path.size() == 0 )
    {

        if (u && TrinityVector3<float>(u->GetPositionX(),u->GetPositionY(),u->GetPositionZ()).dist(TrinityVector3<float>(destx,desty,destz)) <0.05 )
            return true;
        else
            return false;
    }

    if ( u )
    {
        return currwp == wpcount-1 || TrinityVector3<float>(u->GetPositionX(),u->GetPositionY(),u->GetPositionZ())==TrinityVector3<float>(path[path.size()-3],path[path.size()-2],path[path.size()-1]);
    } else {
        return currwp == wpcount-1;
    }

}
/*!
Fa continuare il movimento dall'ultimo punto di pausa
*/
void PathFindingState::UnPause()
{
    TC_LOG_DEBUG("maps", "Movement %p: UnPause()",this);
    statelock.lock();
    mustrecalculate = true;
    pause = false;
    statelock.unlock();
}
void PathFindingState::UpdateDestination(float destx, float desty, float destz, uint32 movementflags, bool fromWorldThread )
{
//  sLog->outString("%s : (%f,%f,%f,%X);\n",__PRETTY_FUNCTION__,destx,desty,destz,movementflags);
    Unit * u = ObjectAccessor::GetObjectInWorld(guid,(Unit*)NULL);
    if ( u )
    {
        if ( fromWorldThread )
            statelock.lock();
        TrinityVector3<float> pos = GetPositionNow();
        lastx = pos.x;
        lasty = pos.y;
        lastz = pos.z;
        targetmflags = movementflags;
        this->destx = destx;
        this->desty = desty;
        this->destz = destz;
        if ( lastx == destx && lasty == desty && lastz == destz )
            TC_LOG_DEBUG("maps", "PF: Warning: attempt to update destination to current position!");
        
        mustrecalculate = true;
        if ( fromWorldThread )
            statelock.unlock();
    }
}

/*!
Questa funzione consente di ottenere la posizione in qualsiasi momento
*/
TrinityVector3<float> PathFindingState::GetPositionNow() //WARNING: Deve essere chiamata con un lock su statelock se non è dal thread di pathfinding
{
    if ( path.size() > 0 && currwp+1 < path.size()/3 )
    {
        //if ( currwp+1 < path.size()/3 )
        // {
        float xi = Interpolate(path[(currwp)*3+0],path[(currwp+1)*3+0],float(currtime-waypointtime[currwp])/float(waypointtime[currwp+1]-waypointtime[currwp]));
        float yi = Interpolate(path[(currwp)*3+1],path[(currwp+1)*3+1],float(currtime-waypointtime[currwp])/float(waypointtime[currwp+1]-waypointtime[currwp]));
        float zi = Interpolate(path[(currwp)*3+2],path[(currwp+1)*3+2],float(currtime-waypointtime[currwp])/float(waypointtime[currwp+1]-waypointtime[currwp]));
        //printf("GetPositionNow(path.size()=%d,waypointtime[currwp]=%d,currwp=%d,waypointx=%f,y=%f,z=%f): %f %f %f\n",path.size(),waypointtime[currwp],currwp,path[(currwp)*3+0],path[(currwp)*3+1],path[(currwp)*3+2],xi,yi,zi);
        return TrinityVector3<float>(xi,yi,zi);
        /*}else{
          float xi = Interpolate(path[(currwp)*3+0],destx,float(currtime)/float(waypointtime[currwp]));
          float yi = Interpolate(path[(currwp)*3+1],desty,float(currtime)/float(waypointtime[currwp]));
          float zi = Interpolate(path[(currwp)*3+2],destz,float(currtime)/float(waypointtime[currwp]));
          printf("GetPositionNow(): %f %f %f\n",xi,yi,zi);
          return TrinityVector3<float>(xi,yi,zi);
        }*/
    } else { //L'ultimo waypoint sarà sempre la destinazione
// printf("GetPositionNow(): %f %f %f\n",lastx,lasty,lastz);

        if ( path.size() < 3 )
            return TrinityVector3<float>(destx,desty,destz);
        else
            return TrinityVector3<float>(path[path.size()-3],path[path.size()-2],path[path.size()-1]);
    }
}

/*PathFindingState* PathFindingMgr::GetPathfindState(Unit* u)
{
  if (!u )
    return NULL;
  boost::mutex::scoped_lock lock(listsmutex);
  if ( paths.find(u->GetGUID()) != paths.end() )
  {
    PathFindingState* tmp = paths[u->GetGUID()];

      return tmp;

  }else{
    for ( std::list<PathFindingState*>::iterator it = addlist.begin(); it != addlist.end(); it++ )
    {
      if ( u->GetGUID() == (*it)->guid )
      {

          return *it;
      }
    }
  }
  return NULL;
}*/

void PathFindingState::UpdateChaseTargetPosition(float x, float y, float z,float mr /* melee range*/)
{
    statelock.lock();
    chasetargetposition.x = x;
    chasetargetposition.y = y;
    chasetargetposition.z = z;
    targetmeleerange = mr;
    statelock.unlock();
}

void PathFindingState::UpdatePetOwnerPosition(float x, float y, float z)
{
    statelock.lock();
    petownerposition.x = x;
    petownerposition.y = y;
    petownerposition.z = z;
    statelock.unlock();
}


PathFindingState::~PathFindingState()
{

}
bool PathFindingMgr::IsValid(PathFindingState* pf)//TODO: Ottimizzare se necessario tramitre l'uso di hashtable
{
    for ( std::list<PathFindingState*>::iterator it = paths.begin(); it != paths.end(); it++ )
    {
        if ( *it == pf )
            return true;
    }
    for ( std::list<PathFindingState*>::iterator it = addlist.begin(); it != addlist.end(); it++ )
    {
        if ( *it == pf )
            return true;
    }
    return false;
}

void PathFindingMgr::RemovePathfind(PathFindingState * pf)
{
    //removelist.push_back(u->GetGUID());
    if ( !IsValid(pf) )
        return;
    PathFindingState * st = pf;
    if ( st )
        st->willdelete = true;
    TC_LOG_DEBUG("maps", "PathFindingMgr::RemovePathfind(%p)",pf);
}
/*std::vector<float> SubdividePath(TrinityVector3<float> startpoint,std::vector<float> origpath,Map * map, float stepsize)
{

  std::vector<float> newpath;
  for ( int i = 0; i < origpath.size()/3; i++)
  {
    float dist;
    if ( i == 0 )
    {
      dist = startpoint.dist(TrinityVector3<float>(origpath[i*3+0],origpath[i*3+1],origpath[i*3+2]));
    }else{
      dist = TrinityVector3<float>(origpath[(i-1)*3+0],origpath[(i-1)*3+1],origpath[(i-1)*3+2]).dist(TrinityVector3<float>(origpath[i*3+0],origpath[i*3+1],origpath[i*3+2]));
      newpath.push_back(origpath[(i-1)*3+0]);
      newpath.push_back(origpath[(i-1)*3+1]);
      newpath.push_back(origpath[(i-1)*3+2]);
    }

    uint32 numpoints = uint32(dist/stepsize);
    for ( uint32 k = 1; k < numpoints+1; k++ )
    {
      float npx = Interpolate(origpath[(i-1)*3+0],origpath[(i)*3+0],float(k)/float(numpoints));
      float npy = Interpolate(origpath[(i-1)*3+1],origpath[(i)*3+1],float(k)/float(numpoints));
      float npz_ = Interpolate(origpath[(i-1)*3+2],origpath[(i)*3+2],float(k)/float(numpoints));
      float npz = map->GetHeight(npx,npy,npz+2.0f,true,50.0f);
      newpath.push_back(npx);
      newpath.push_back(npy);
      newpath.push_back(npz);
    }
  }
  return newpath;
}*/

#define GetX(v,index) v[(index)*3+0]
#define GetY(v,index) v[(index)*3+1]
#define GetZ(v,index) v[(index)*3+2]
#define Append3(v,v2,index) v.push_back(v2[index*3+0]);v.push_back(v2[index*3+1]);v.push_back(v2[index*3+2]);
/*
Questa funzione elimina i waypoint non necessari ( ad esempio in linea d'aria )
*/
std::vector<float> SimplifyPath(std::vector<float> &origpath,std::vector<float> &newpath)
{
    newpath.clear();
    if ( origpath.size() == 0 )
        return newpath;
    float dist1,dist2,ratio,delta;
    for ( int i = 0; i < origpath.size()/3; i++ )
    {
        if ( i == 0 )
        {
            Append3(newpath,origpath,i)
            continue;
        }
        if ( i+1 >= origpath.size()/3 )
        {
            Append3(newpath,origpath,i)
            continue;
        }

        TrinityVector3<float> vp0(GetX(origpath,i-1),GetY(origpath,i-1),GetZ(origpath,i-1));
        TrinityVector3<float> vpcentrale(GetX(origpath,i),GetY(origpath,i),GetZ(origpath,i));
        TrinityVector3<float> vp1(GetX(origpath,i+1),GetY(origpath,i+1),GetZ(origpath,i+1));
        TrinityVector3<float> vDir = (vpcentrale-vp0).normalize();
        dist1 = vp0.dist(vpcentrale);
        dist2 = vp0.dist(vp1);
        TrinityVector3<float> t = vp0+vDir*dist2; //Il terzo punto seguendo l'angolazione 3d descritta dai punti vp0 e vpcentrale
        delta = t.dist(vp1);
        if ( delta > 0.07 )
        {
            TC_LOG_DEBUG("maps", "WP %u : %f",uint32(newpath.size()/3),delta);
            Append3(newpath,origpath,i)
        }
    }
    for ( int i = 0; i < newpath.size()/3; i++ )
    {
        //TC_LOG_DEBUG("maps", "SimplifyPath: WAYPOINT %d : %f %f %f",i,newpath[i*3+0],newpath[i*3+1],newpath[i*3+2]);
    }
    //TC_LOG_DEBUG("maps", "SimplifyPath: origwaypoints: %d newwaypoints: %d\n",origpath.size()/3,newpath.size()/3);
    std::vector<float> res;
    if ( newpath.size() < 6 )
        return res;
    if ( newpath.size() < 9 )
        return newpath;
    res.resize(newpath.size()-3);
    memcpy(&res[0],&newpath[3],res.size()*sizeof(float));//Remove first waypoint to avoid glitches
    return res;


}

void PathFindingState::Calculate(Map* m,Unit * u)
{
    //sLog->outString("void PathFindingState::Calculate(Map* m,Unit * u) : speed=%f\n",speed);
    if (!u)
        return;
    //sLog->outError("PathFindingState::Calculate()");
    //Le probabilità che si verifichi che sono in aggiornamento le coordinate sono praticamente 0 e nel caso succedesse ci sarà solo qualche lievissimo glitch
    lastx = u->GetPositionX();
    lasty = u->GetPositionY();
    lastz = u->GetPositionZ();

    if ( lastx == destx && lastz == destz && desty == lasty )
    {
        
        path.resize(0);
        path2send.clear();
        wpcount = 0;
        arrived = true;
        calculated = true;
        currtime = 0;
        currwp = 0;
        TC_LOG_DEBUG("maps","<%s> Percorso (%f,%f,%f) -> (%f,%f,%f) Calcolato, Waypoint: %u , Stato: %d",u->GetName().c_str(),lastx,lasty,lastz,destx,desty,destz,uint32(path.size()/3),status);
        return;



    }
    bool lospath = false;
    pathfindResult res;
    std::vector<float> pathtemp;
    path2send.clear();
    if ( u->GetUnitMovementFlags() & ( MOVEMENTFLAG_DISABLE_GRAVITY | MOVEMENTFLAG_FLYING | MOVEMENTFLAG_CAN_FLY ) )//I mob volanti non hanno pathfinding
    {
        //TC_LOG_DEBUG("maps", "\033[34mUnit %p flying\033[0m",u);
        wpcount = 2;
        pathtemp.resize(6);
        pathtemp[0] = lastx;
        pathtemp[1] = lasty;
        pathtemp[2] = lastz;
        pathtemp[3] = destx;
        pathtemp[4] = desty;
        pathtemp[5] = destz;
        currwp = 0;

        currtime = 0;
        arrived = false;
        calculated = true;
        lastposupdate = 0;
    } else {
        uint16 incflags = 0x0000;
        uint16 excflags = 0x0000;
        if ( u->GetTypeId() == TYPEID_PLAYER || u->IsControlledByPlayer() )
            incflags |= 0xFFFF; //I player e i pet possono andare ovunque dove l'angolo è < 45 gradi
        Creature * c;
        if ( c = u->ToCreature() )
        {
            if ( c->GetCreatureTemplate()->InhabitType & INHABIT_WATER )
            {
                //printf("Pathfind: Creature WATER!\n");
                incflags |= NAV_WATER;
            }
            if ( c->GetCreatureTemplate()->InhabitType & INHABIT_GROUND)
            {
                incflags |= NAV_GROUND | NAV_SHALLOW_WATER ;
                //printf("Pathfind: Creature GROUND!\n");
            }
            if ( c->GetCreatureTemplate()->InhabitType & INHABIT_AIR)
            {
                incflags |= 0xFFFF;
            }
        }

        if ( u->IsControlledByPlayer() )
        {

            res = m->Pathfind(this,lastx,lasty,lastz,destx,desty,destz,incflags,excflags,3.5f);
        }
        else
        {
            petownerposition.x = NAN;
            res = m->Pathfind(this,lastx,lasty,lastz,destx,desty,destz,incflags,excflags,3.5f);
        }
        if ( res.path )
        {
            pathtemp.resize(res.waypointcount*3);
            memcpy(&pathtemp[0],res.path,res.waypointcount*3*sizeof(float));
            delete res.path;
        }
        wpcount = res.waypointcount;
        if ( c = u->ToCreature() )
        {
            if ( c->GetCreatureTemplate()->InhabitType & INHABIT_AIR && (res.result == PATHFIND_CANT_FIND_END_NAVMESH || res.result == PATHFIND_CANT_FIND_START_NAVMESH) )
            {
                wpcount = 2;
                pathtemp.resize(6);
                pathtemp[0] = lastx;
                pathtemp[1] = lasty;
                pathtemp[2] = lastz;
                pathtemp[3] = destx;
                pathtemp[4] = desty;
                pathtemp[5] = destz;
                currwp = 0;

                currtime = 0;
                arrived = false;
                calculated = true;
                lastposupdate = 0;
            }
        }
        
        
        
        if ( !pathtemp.size() )
        {
            //sLog->outError("Pathfinding failed. (Start: %f %f %f , Dest: %f %f %f) Died=%s",lastx,lasty,lastz,destx,desty,destz,u->isAlive() ? "false" : "true" );
            wpcount = 2;
            pathtemp.resize(6);
            pathtemp[0] = lastx;
            pathtemp[1] = lasty;
            pathtemp[2] = lastz;
            pathtemp[3] = destx;
            pathtemp[4] = desty;
            pathtemp[5] = destz;
            lospath = true;
        }
        origwpcount = pathtemp.size()/3;
        
    }
    path2send = SimplifyPath(pathtemp,path);
    TrinityVector3<double> last_point;
//     printf("Ischarge=%d\n",(int)isCharge);
    if ( isCharge )
    {
        bool finished_path = NewContactPoint(TrinityVector3<double>(destx,desty,destz),MELEE_RANGE-0.05,path2send,&last_point);
        if ( finished_path )
        {
            destx = last_point.x;
            desty = last_point.y;
            destz = last_point.z;
        }
    }
    currwp = 0;


    currtime = 0;
    arrived = false;
    calculated = true;
    lastposupdate = 0;

    waypointtime.clear();

    wpcount = path.size()/3;

    uint32 time = 0;
    int i2;
    waypointtime.push_back(time);//Il primo waypoint è il punto di partenza e per raggiungerlo ci vogliono 0 millisecondi
    for ( int i = 1; i < wpcount; i++ )
    {

        float timesecs = 0.0f;
        i2 = i-1;
        
        G3D::Vector3 prev(path[i2*3+0],path[i2*3+1],path[i2*3+2]);
        G3D::Vector3 curr(path[i*3+0],path[i*3+1],path[i*3+2]);
        timesecs = (prev-curr).magnitude()/speed;
        /*if ( i2 < wpcount)
            timesecs = sqrt((path[i*3+0]-path[i2*3+0])*(path[i*3+0]-path[i2*3+0])+(path[i*3+1]-path[i2*3+1])*(path[i*3+1]-path[i2*3+1])+(path[i*3+2]-path[i2*3+2])*(path[i*3+2]-path[i2*3+2]))/float(speed);
        else
            timesecs = sqrt((path[i*3+0]-destx)*(path[i*3+0]-destx)+(path[i*3+1]-desty)*(path[i*3+1]-desty)+(path[i*3+2]-destz)*(path[i*3+2]-destz))/float(speed);*/
        time += uint32(timesecs*1000.0f);
        waypointtime.push_back(time);
    }
    status = PATHFINDINGSTATUS_FINAL;
    if ( origwpcount == MAX_SMOOTH )
        status = PATHFINDINGSTATUS_PARTIAL;
    if ( origwpcount < MAX_SMOOTH )
    {
        TrinityVector3<float> endwaypoint(path[path.size()-3],path[path.size()-2],path[path.size()-1]);
        TrinityVector3<float> destination(destx,desty,destz);
        float ztoll = fabs(endwaypoint.z-destination.z);
        if ( ztoll > 6.0 )
            ztoll = 0;
        if ( endwaypoint.dist(destination) > STEP_SIZE+0.2f+ztoll )
        {
            TC_LOG_DEBUG("maps","Dest Unreachable ( %f (%f , %f ) )\n",endwaypoint.dist(destination),endwaypoint.z,destination.z);
            
            status = PATHFINDINGSTATUS_DEST_UNREACHABLE;
            if ( isCharge && origwpcount > 1 )
                status = PATHFINDINGSTATUS_FINAL;
            if ( !isnan(chasetargetposition.x) && chasetargetposition.dist(endwaypoint) <= targetmeleerange+u->GetMeleeReach() ) // Se è raggiungibile inr ange melee considera valido il percorso
            {
                status = PATHFINDINGSTATUS_FINAL;
            }
        }
    }

    if (  (res.result == PATHFIND_CANT_FIND_END_NAVMESH || res.result == PATHFIND_CANT_FIND_START_NAVMESH ) && targetmflags && targetmflags & ( MOVEMENTFLAG_FLYING ))
    {
        Creature * attacker = u->ToCreature();//Non può essere deletato da un altro thread perché è stato acquisito il lock
        if ( attacker )
        {
            if ( !(attacker->GetCreatureTemplate()->InhabitType & INHABIT_AIR ))
            {
                status = PATHFINDINGSTATUS_DEST_UNREACHABLE;
            }

        }
    }
    
    if ( status == PATHFINDINGSTATUS_DEST_UNREACHABLE )
    {
        if (Creature * c = u->ToCreature() )
        {
            if ( c->GetCreatureTemplate()->InhabitType & INHABIT_AIR)
            {
                wpcount = 2;
                pathtemp.resize(6);
                pathtemp[0] = lastx;
                pathtemp[1] = lasty;
                pathtemp[2] = lastz;
                pathtemp[3] = destx;
                pathtemp[4] = desty;
                pathtemp[5] = destz;
                path2send.resize(6);
                path2send[0] = destx;//Con 1 waypoint solo si bugga la spline
                path2send[1] = desty;
                path2send[2] = destz;
                path2send[3] = destx;
                path2send[4] = desty;
                path2send[5] = destz;
                currwp = 0;

                currtime = 0;
                arrived = false;
                calculated = true;
                lastposupdate = 0;
                status = PATHFINDINGSTATUS_FINAL;
            }
        }
        
    }
    /*if ( u->isPet() && status == PATHFINDINGSTATUS_DEST_UNREACHABLE )
        status = PATHFINDINGSTATUS_BROKENNAVMESH;*/

    if ( debug )
        m->mtcalls->post(boost::bind(&PathViewer::UpdatePath,debug,path));

    TC_LOG_DEBUG("maps","<%s> Percorso (%f,%f,%f) -> (%f,%f,%f) Calcolato, Waypoint: %u , Stato: %d",u->GetName().c_str(),lastx,lasty,lastz,destx,desty,destz,uint32(path.size()/3),status);
}

uint32 idcounter = 0;
PathFindingState* PathFindingMgr::AddPathfind(Unit* u, float destx, float desty, float destz, float speed, bool isCharge)
{
    TC_LOG_DEBUG("maps", "PathFindingMgr::AddPathfind(%p,%f;%f,%f)",u,destx,desty,destz);
    //boost::mutex::scoped_lock lock(listsmutex);

    if (!u )
        return NULL;

    /*if ( paths.find(u->GetGUID()) != paths.end() ) //Controlla se la Unit ha già un percorso
      return paths[u->GetGUID()];*/

    PathFindingState * state = new PathFindingState;
    state->calculated = false;//Verrà calcolato sul thread dedicato
    state->unittypeid = u->GetTypeId();
    state->guid = u->GetGUID();
    state->destx = destx;
    state->desty = desty;
    state->willdelete = false;
    state->destz = destz;
    state->id = ++idcounter;
    state->map = map;
    state->lastx = u->GetPositionX();
    state->lasty = u->GetPositionY();
    state->lastz = u->GetPositionZ();
    state->arrived = false;
    state->pause = false;
    state->speed = speed;
    state->pmgr = this;
    state->status = 0;
    state->debug = 0;
    state->isCharge = isCharge;
    state->petownerposition.x = G3D::nan();
    state->petownerposition.y = G3D::nan();
    state->petownerposition.z = G3D::nan();
    state->chasetargetposition.x = G3D::nan();
    state->chasetargetposition.y = G3D::nan();
    state->chasetargetposition.z = G3D::nan();
    state->facingTarget = 0;
    //if ( u->GetMotionMaster()->pathfindingdebug ) TODO: Reimplementare
    //  state->debug = new PathViewer(u);
    addlist.push_back(state);
    return state;
}



double getcurrenttime()
{
    struct timeval t;
    gettimeofday(&t,NULL);
    double ti = 0.0;
    ti = t.tv_sec;
    ti += t.tv_usec/1000000.0;
    return ti;
}


inline bool inRange(const float* v1, const float* v2, const float r, const float h)
{
    const float dx = v2[0] - v1[0];
    const float dy = v2[1] - v1[1];
    const float dz = v2[2] - v1[2];
    return (dx*dx + dz*dz) < r*r && fabsf(dy) < h;
}


static bool getSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos,
                                                   const float minTargetDist,
                                                   const dtPolyRef* path, const int pathSize,
                                                   float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef,
                                                   float* outPoints = 0, int* outPointCount = 0)                                                         
{
        // Find steer target.
        static const int MAX_STEER_POINTS = 3;
        float steerPath[MAX_STEER_POINTS*3];
        unsigned char steerPathFlags[MAX_STEER_POINTS];
        dtPolyRef steerPathPolys[MAX_STEER_POINTS];
        int nsteerPath = 0;
        navQuery->findStraightPath(startPos, endPos, path, pathSize,
                                                           steerPath, steerPathFlags, steerPathPolys, &nsteerPath, MAX_STEER_POINTS);
        if (!nsteerPath)
                return false;
                
        if (outPoints && outPointCount)
        {
                *outPointCount = nsteerPath;
                for (int i = 0; i < nsteerPath; ++i)
                        dtVcopy(&outPoints[i*3], &steerPath[i*3]);
        }

        
        // Find vertex far enough to steer to.
        int ns = 0;
        while (ns < nsteerPath)
        {
                // Stop at Off-Mesh link or when point is further than slop away.
                if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
                        !inRange(&steerPath[ns*3], startPos, minTargetDist, 1000.0f))
                        break;
                ns++;
        }
        // Failed to find good point to steer to.
        if (ns >= nsteerPath)
                return false;
        
        dtVcopy(steerPos, &steerPath[ns*3]);
        steerPos[1] = startPos[1];
        steerPosFlag = steerPathFlags[ns];
        steerPosRef = steerPathPolys[ns];
        
        return true;
}


static int fixupCorridor(dtPolyRef* path, const int npath, const int maxPath,
                                                 const dtPolyRef* visited, const int nvisited)
{
        int furthestPath = -1;
        int furthestVisited = -1;
        
        // Find furthest common polygon.
        for (int i = npath-1; i >= 0; --i)
        {
                bool found = false;
                for (int j = nvisited-1; j >= 0; --j)
                {
                        if (path[i] == visited[j])
                        {
                                furthestPath = i;
                                furthestVisited = j;
                                found = true;
                        }
                }
                if (found)
                        break;
        }

        // If no intersection found just return current path. 
        if (furthestPath == -1 || furthestVisited == -1)
                return npath;
        
        // Concatenate paths.   

        // Adjust beginning of the buffer to include the visited.
        const int req = nvisited - furthestVisited;
        const int orig = rcMin(furthestPath+1, npath);
        int size = rcMax(0, npath-orig);
        if (req+size > maxPath)
                size = maxPath-req;
        if (size)
                memmove(path+req, path+orig, size*sizeof(dtPolyRef));
        
        // Store visited
        for (int i = 0; i < req; ++i)
                path[i] = visited[(nvisited-1)-i];                              
        
        return req+size;
}
bool Map::NavMeshLOS(float startx, float starty, float startz, float endx, float endy, float endz, TrinityVector3<float>* coll_point)
{
    boost::mutex::scoped_lock lock(navmeshmutex);
    float startPos[3]               = { starty, startz, startx };
    float endPos[3]                 = { endy, endz, endx };
    float mPolyPickingExtents[3]    = { 2.0f, 4.0f, 2.0f };
    if ( !m_navMesh )
        return true;
    dtNavMeshQuery* navQuery = new dtNavMeshQuery;
    if ( !navQuery->init(m_navMesh,1000) )
    {
        TC_LOG_ERROR("general", "Impossibile inizializzare dtNavMeshQuery Map::NavMeshLOS(%f,%f,%f,%f,%f,%f)",startx,starty,startz,endx,endy,endz);

        delete navQuery;
        return true;
    }
    dtQueryFilter * mPathFilter = new dtQueryFilter();
    mPathFilter->setIncludeFlags(0xFFFF);
    mPathFilter->setExcludeFlags(0x0000);
    dtPolyRef mStartRef;
    navQuery->findNearestPoly(startPos,mPolyPickingExtents,mPathFilter,&mStartRef,NULL);
    if (mStartRef != 0 )
    {
        float t;
        float hitnormal[3];
        dtPolyRef pathres[64];
        int visitedp;
        navQuery->raycast(mStartRef,startPos,endPos,mPathFilter,&t,hitnormal,pathres,&visitedp,64);
        if ( t == FLT_MAX )
        {
            delete navQuery;
            delete mPathFilter;
            return true;
        }
        else
        {
            TrinityVector3<float> start(startx, starty, startz);
            TrinityVector3<float> end(endx, endy, endz);
            TrinityVector3<float> diff = end-start;
            TrinityVector3<float> res = start+diff*t;
            *coll_point = res;
            delete navQuery;
            delete mPathFilter;
            return res == end;
        }



    }
    delete navQuery;
    delete mPathFilter;
    return true;
}
std::vector<G3D::Vector3> Map::PathFindDirect(Unit * moving, G3D::Vector3 start , G3D::Vector3 end)
{
    std::vector<G3D::Vector3> path;
    PathFindingState * dummypf = new PathFindingState();
    dummypf->petownerposition.x = NAN;
    dummypf->petownerposition.y = NAN;
    dummypf->petownerposition.z = NAN;
    uint16 incflags = 0x0000;
    uint16 excflags = 0x0000;
    if ( moving->GetTypeId() == TYPEID_PLAYER || moving->IsControlledByPlayer() )
        incflags |= 0xFFFF; //I player e i pet possono andare ovunque dove l'angolo è < 45 gradi
    Creature * c;
    if ( c = moving->ToCreature() )
    {
        if ( c->GetCreatureTemplate()->InhabitType & INHABIT_WATER )
        {
            //printf("Pathfind: Creature WATER!\n");
            incflags |= NAV_WATER;
        }
        if ( c->GetCreatureTemplate()->InhabitType & INHABIT_GROUND)
        {
            incflags |= NAV_GROUND | NAV_SHALLOW_WATER ;
            //printf("Pathfind: Creature GROUND!\n");
        }
        if ( c->GetCreatureTemplate()->InhabitType & INHABIT_AIR)
        {
            incflags |= 0xFFFF;
        }
    }
    pathfindResult res = Pathfind(dummypf,start.x,start.y,start.z,end.x,end.y,end.z,incflags,excflags);
    TrinityVector3<float> lastwp;
    if ( res.result == PATHFIND_OK )
    {
        for ( int i = 0; i < res.waypointcount; i++ )
        {
            path.push_back(G3D::Vector3(res.path[i*3+0],res.path[i*3+1],res.path[i*3+2]));
	    lastwp.x = res.path[i*3+0];
	    lastwp.y = res.path[i*3+1];
	    lastwp.z = res.path[i*3+2];
        }
        delete res.path;
    }
    if ( TrinityVector3<float>(end.x,end.y,end.z).dist(lastwp) > MELEE_RANGE )
	path.clear();
    return path;
}

void CorrectPetPosition(PathFindingState* pfstate, dtNavMeshQuery * navQuery, float * petownerpos, float * endPos , dtQueryFilter * mPathFilter, float * mPolyPickingExtents, dtPolyRef * mPathResults, unsigned int maxpathresults, dtPolyRef& mEndRef )
{
    float t;
    float hitNormal[3];

    dtPolyRef PetOwnerRef;
    navQuery->findNearestPoly(petownerpos,mPolyPickingExtents,mPathFilter,&PetOwnerRef,NULL);
    if ( PetOwnerRef  != 0)
    {
        /*int res = navQuery->raycast(PetOwnerRef,popc,endPos,mPathFilter,t,hitNormal,mPathResults,maxpathresults);
        TC_LOG_DEBUG("maps", "RAYCAST: res=%d , t=%f ",res,t);
        if ( res != 0 && t < FLT_MAX && t < 100.0)
        {
            TC_LOG_DEBUG("maps", "Il pet non è in LOS sulla navmesh con il proprietario , correzione destinazione");
            TrinityVector3<float> vDir(hitNormal[2],hitNormal[0],hitNormal[1]);
            TrinityVector3<float> ownerPos(petownerpos[2],petownerpos[0],petownerpos[1]);
            TrinityVector3<float> newDestifFail = ownerPos+(vDir*t);
            TrinityVector3<float> diff = TrinityVector3<float>(endPos[2],endPos[0],endPos[1])-ownerPos;
            TrinityVector3<float> newDest_DX = ownerPos-diff;
            float tDX;
            float hitNormalDX[3];
            float endPosDX[3] = { newDest_DX.y, newDest_DX.z, newDest_DX.x };
            int resDX = navQuery->raycast(PetOwnerRef,popc,endPosDX,mPathFilter,tDX,hitNormalDX,mPathResults,maxpathresults);
            TC_LOG_DEBUG("maps", "Tentativo di spostare il pet a destra");
            dtPolyRef oldEndRef = mEndRef;
            if ( resDX != 0 && !isinf(tDX) && tDX < 100.0)
            {
                TC_LOG_DEBUG("maps", "Destra dell'owner del pet occupata!");
                endPos[0] = petownerpos[0];
                endPos[1] = petownerpos[1];
                endPos[2] = petownerpos[2];


            } else {
                endPos[0] = newDest_DX.y;
                endPos[1] = newDest_DX.z;
                endPos[2] = newDest_DX.x;

            }

            /*endPos[0] = newDest.x;
            endPos[1] = newDest.y;
            endPos[2] = newDest.z;*//*
mEndRef = navQuery->findNearestPoly(endPos,mPolyPickingExtents,mPathFilter,0);
if ( mEndRef == 0 )
{
TC_LOG_DEBUG("maps", "Impossibile trovare la referenza di fine percorso per la nuova destinazione pet");
mEndRef = oldEndRef;

}*/
        /*}else if ( !mEndRef ) { //Il pet si trova su una zona non valida
            TrinityVector3<float> vDir(hitNormal[2],hitNormal[0],hitNormal[1]);
            TrinityVector3<float> ownerPos(petownerpos[2],petownerpos[0],petownerpos[1]);
            TrinityVector3<float> newDestifFail = ownerPos+(vDir*t);
            TrinityVector3<float> diff = TrinityVector3<float>(endPos[2],endPos[0],endPos[1])-ownerPos;
            TrinityVector3<float> newDest_DX = ownerPos-diff;
            float tDX;
            float hitNormalDX[3];
            float endPosDX[3] = { newDest_DX.y, newDest_DX.z, newDest_DX.x };
            int resDX = navQuery->raycast(PetOwnerRef,popc,endPosDX,mPathFilter,tDX,hitNormalDX,mPathResults,maxpathresults);
            TC_LOG_DEBUG("maps", "Tentativo di spostare il pet a destra");
            dtPolyRef oldEndRef = mEndRef;
            if ( resDX != 0 && !isinf(tDX) && tDX < 100.0)
            {
                TC_LOG_DEBUG("maps", "Destra dell'owner del pet occupata!");
                endPos[0] = petownerpos[0];
                endPos[1] = petownerpos[1];
                endPos[2] = petownerpos[2];
            } else {
                endPos[0] = newDest_DX.y;
                endPos[1] = newDest_DX.z;
                endPos[2] = newDest_DX.x;

            }

            /*endPos[0] = newDest.x;
            endPos[1] = newDest.y;
            endPos[2] = newDest.z;*//*
mEndRef = navQuery->findNearestPoly(endPos,mPolyPickingExtents,mPathFilter,0);
if ( mEndRef == 0 )
{
TC_LOG_DEBUG("maps", "Impossibile trovare la referenza di fine percorso per la nuova destinazione pet");
mEndRef = oldEndRef;

}*/
        float maxdist = TrinityVector3<float>(petownerpos[0],petownerpos[1],petownerpos[2]).dist(TrinityVector3<float>(endPos[0],endPos[1],endPos[2]));
        if ( !mEndRef )
        {
            float pp2[3] = { 10.0 , 10.0 , 10.0 };

            navQuery->findNearestPoly(endPos,pp2,mPathFilter,&mEndRef,NULL);
            TC_LOG_DEBUG("maps","endRef2");
        }
        if (!mEndRef)
        {
            TC_LOG_DEBUG("maps","endref2->no");
            mEndRef = PetOwnerRef;
            return;
        }
        float hitpos[3];
        float hitdist;
        navQuery->findDistanceToWall(PetOwnerRef,petownerpos,maxdist+0.1,mPathFilter,&hitdist,hitpos,hitNormal);
        if ( hitdist < maxdist )
        {
            TC_LOG_DEBUG("maps","Correzione pet ( dWall ) (%f , %f , %f )", hitpos[2], hitpos[0], hitpos[1]);
            memcpy(endPos,hitpos,sizeof(float)*3);

        }
        navQuery->findNearestPoly(endPos,mPolyPickingExtents,mPathFilter,&mEndRef,NULL);
    } else {
        TC_LOG_DEBUG("maps", "Impossibile trovare la ref del pet owner (%s)!",pfstate->petownerposition.as_str().c_str());

    }


}
#define ERRCHKDT(x) \
    if ( dtStatusDetail(s,x) ) \
        errstr << #x << "|";
inline void PrintDetourError(dtStatus s)
{
    if ( dtStatusFailed(s) )
    {
        std::stringstream errstr;
        errstr << "Pathfinding: ";
        ERRCHKDT(DT_WRONG_MAGIC)
        ERRCHKDT(DT_WRONG_VERSION)
        ERRCHKDT(DT_OUT_OF_MEMORY)
        ERRCHKDT(DT_BUFFER_TOO_SMALL)
        ERRCHKDT(DT_INVALID_PARAM)
        ERRCHKDT(DT_OUT_OF_NODES)
        
        TC_LOG_ERROR("general", "%s",errstr.str().c_str());
        
    }
}

bool isUnitOnGround(Unit * u)
{
    float pos[3] = {u->GetPositionY(), u->GetPositionZ(), u->GetPositionX()};
    float mPolyPickingExtents[3]    = { 1.5f, 1.5f, 1.5f };
    dtQueryFilter * mPathFilter = new dtQueryFilter();
    mPathFilter->setIncludeFlags(0xFFFF);
    mPathFilter->setExcludeFlags(0x0000);
    dtNavMeshQuery* navQuery = new dtNavMeshQuery;
    dtNavMesh* m_navMesh = u->GetMap()->m_navMesh;
    if ( m_navMesh && navQuery ) {
        if(navQuery->init(m_navMesh,/*numero nodi massimo da visitare... lo metterei nel config..*/1000)) {
            dtPolyRef mStartRef;
            dtStatus s;
            s = navQuery->findNearestPoly(pos,mPolyPickingExtents,mPathFilter,&mStartRef,NULL);
            if ( mStartRef )
            {
                delete mPathFilter;
                delete navQuery;
                return true;

            }
        }
        delete mPathFilter;
        delete navQuery;
        return false;
    }
    delete mPathFilter;
    if ( navQuery ) delete navQuery;
    return false;
}

/*
 * Commenti su alcuni TODO by kamir:
 * Velocizzare il codice:
 * Soluzione: La maggior parte delle vole in cui questo codice viene utilizzato il mob � perfettamente in linea con il target
 * Lanciare un raycast sul punto di destinazione prima di chiamare le funzioni di pathfind.
 * I mob camminano troppo raso-muro:
 * Soluzione: circoscrivere un quadrato alla circonferenza del mob
 * Con i vertici di questo quadrato utilizzare il raycast sul prossimo punto del path
 * Se il raycast ha successo su ogni vertice, muovere il mob verso la posizione,
 * altrimenti vedere quale vertice ha fallito, e spostare quel vertice nella direzione opposta dello stesso
 * (esempio se il vertice fallito � quello in alto a sinistra e il punto � in alto a destra, spostare il mob verso destra)
 * se dopo un certo numero di try il mob non esce da questa posizione indefinita, farlo entrare in evade.
 * o forzarne ugualmente il cammino.
 */


pathfindResult Map::Pathfind(PathFindingState * pfstate, float srcx, float srcy, float srcz, float destx, float desty, float destz,uint16 inclflags,uint16 exclflags,float searchdist)
{
    boost::mutex::scoped_lock lock(navmeshmutex);
    pathfindResult result;
    result.path = NULL;
    float startPos[3]               = { srcy, srcz, srcx };
    float endPos[3]                 = { desty, destz, destx };
    float petownerpos[3]            = { pfstate->petownerposition.y , pfstate->petownerposition.z , pfstate->petownerposition.x };
    float mPolyPickingExtents[3]    = { searchdist, 5.0f, searchdist };
    double tstart = getcurrenttime();
    dtQueryFilter * mPathFilter = new dtQueryFilter();
    Position pos = Position();
    /*pos.m_positionX = destx;
    pos.m_positionY = desty;
    pos.m_positionZ = destz;*/
    TC_LOG_DEBUG("maps", "Map::Pathfind(%f,%f,%f,%f,%f,%f)",srcx,srcy,srcz,destx,desty,destz);
    mPathFilter->setIncludeFlags(inclflags);
    mPathFilter->setExcludeFlags(exclflags);
    //dtNavMesh* myNavMesh = m_navMesh;
    dtNavMeshQuery* navQuery = new dtNavMeshQuery;
    if ( m_navMesh && navQuery ) {
        if(navQuery->init(m_navMesh,/*numero nodi massimo da visitare... lo metterei nel config..*/1000)) {
            dtPolyRef mStartRef;
            dtStatus s;
            s = navQuery->findNearestPoly(startPos,mPolyPickingExtents,mPathFilter,&mStartRef,NULL); // this maybe should be saved on mob for later
            mPathFilter->setIncludeFlags(0xFFFF);//Per la destinazione è  necessario includere tutti i flag in modo da capire se è valida o no
            mPathFilter->setExcludeFlags(0);
            PrintDetourError(s);
            dtPolyRef mEndRef;
            s = navQuery->findNearestPoly(endPos,mPolyPickingExtents,mPathFilter,&mEndRef,NULL); // saved on player? probably waste since player moves t
            PrintDetourError(s);
            mPathFilter->setIncludeFlags(inclflags);
            mPathFilter->setExcludeFlags(exclflags);
            if (!mEndRef && mStartRef && !isnan(pfstate->petownerposition.x) )
            {
                unsigned int maxpathresults = sWorld->getIntConfigMT(CONFIG_PATHFINDING_MAX_PATH_SIZE);
                dtPolyRef * mPathResults = (dtPolyRef*)malloc(sizeof(dtPolyRef)*maxpathresults);
                TC_LOG_DEBUG("maps", "Destinazione pet non esistente");
                CorrectPetPosition(pfstate,navQuery,petownerpos,endPos,mPathFilter,mPolyPickingExtents,mPathResults,maxpathresults,mEndRef);
                free(mPathResults);
            }
            if (mStartRef != 0 && mEndRef != 0)
            {


                unsigned int maxpathresults = sWorld->getIntConfigMT(CONFIG_PATHFINDING_MAX_PATH_SIZE);
                dtPolyRef * mPathResults = (dtPolyRef*)malloc(sizeof(dtPolyRef)*maxpathresults);
                if ( !isnan(pfstate->petownerposition.x) )
                {
                    CorrectPetPosition(pfstate,navQuery,petownerpos,endPos,mPathFilter,mPolyPickingExtents,mPathResults,maxpathresults,mEndRef);
                }
                /*sLog->outError("Raycast result = %d",res);
                sLog->outError("T = %f",t);
                sLog->outError("hitNormal = %f , %f , %f",hitNormal[0],hitNormal[1],hitNormal[2]);
                if (t == FLT_MAX)
                {
                        result.result = PATHFIND_OK;
                        result.path = new float[3];
                        result.path[0] = destx;
                        result.path[1] = desty;
                        result.path[2] = destz;
                        result.waypointcount = 1;
                        sLog->outError("Raycast avvenuto con successo");
                        delete navQuery;
                        return result;
                } TODO: Da risultati senza senso, inoltre non conviene assolutamente farlo , in quanto l'operazione più pesante è trovare i poligoni di partenza e fine
                */
                int mNumPathResults;
                pfstate->destx = endPos[2];//Serve in caso di correzione della posizione del pet e dovrebbe evitare i glitch che fanno girare su se stesso il pet
                pfstate->desty = endPos[0];
                pfstate->destz = endPos[1];
                dtStatus s2 = navQuery->findPath(mStartRef,mEndRef,startPos, endPos, mPathFilter ,mPathResults,&mNumPathResults,maxpathresults);
                PrintDetourError(s2);
                if(mNumPathResults <= 0) {
                    TC_LOG_DEBUG("maps", "Pathfinding fallito.");
                    result.result = PATHFIND_NO_PATHS_TO_TARGET;
                    delete navQuery;
                    free(mPathResults);
                    return result;
                }
                TC_LOG_DEBUG("maps", "Pathfinding: MaxWP: %d MaxPolys:%d",sWorld->getIntConfigMT(CONFIG_PATHFINDING_MAX_WAYPOINTS),sWorld->getIntConfigMT(CONFIG_PATHFINDING_MAX_PATH_SIZE));
                //float actualpath[3*sWorld->getIntConfigMT(CONFIG_PATHFINDING_MAX_WAYPOINTS)];
                unsigned char* flags = 0;
                dtPolyRef* polyrefs = 0;


                int MAX_POLYS = 1000;

                dtPolyRef * polys = new dtPolyRef[MAX_POLYS];
                memcpy(polys, mPathResults, sizeof(dtPolyRef)*mNumPathResults);
                int npolys = mNumPathResults;







                /*int mNumPathPoints = navQuery->findStraightPath(startPos, endPos,mPathResults, mNumPathResults, actualpath, flags, polyrefs,sWorld->getIntConfig(CONFIG_PATHFINDING_MAX_WAYPOINTS));
                */
                float iterPos[3], targetPos[3];

                navQuery->closestPointOnPolyBoundary(mStartRef, startPos, iterPos);
                navQuery->closestPointOnPolyBoundary(polys[npolys-1], endPos, targetPos);

                //static const float STEP_SIZE = 0.5f;
                static const float SLOP = 0.01f;

                int m_nsmoothPath = 0;
                float * m_smoothPath = (float*)malloc(3*MAX_SMOOTH*sizeof(float));

                dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
                m_nsmoothPath++;

                // Move towards target a small advancement at a time until target reached or
                // when ran out of memory to store the path.
                while (npolys && m_nsmoothPath < MAX_SMOOTH)
                {
                    // Find location to steer towards.
                    float steerPos[3];
                    unsigned char steerPosFlag;
                    dtPolyRef steerPosRef;
                    ;
                    if (!getSteerTarget(navQuery, iterPos, targetPos, SLOP,
                                        polys, npolys, steerPos, steerPosFlag, steerPosRef))
                        break;
                    ;
                    bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
                    bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
                    ;
                    // Find movement delta.
                    float delta[3], len;
                    ;
                    dtVsub(delta, steerPos, iterPos);
                    len = dtSqrt(dtVdot(delta,delta));
                    ;
                    // If the steer target is end of path or off-mesh link, do not move past the location.
                    if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
                    {
                        len = 1;
                    } else {
                        len = STEP_SIZE / len;
                    }
                    ;
                    float moveTgt[3];
                    dtVmad(moveTgt, iterPos, delta, len);

                    // Move
                    float result[3];
                    dtPolyRef visited[16];

                    int nvisited;
                    navQuery->moveAlongSurface(polys[0], iterPos, moveTgt, mPathFilter,result,visited,&nvisited,16);

                    ;

                    npolys = fixupCorridor(polys, npolys, MAX_POLYS, visited, nvisited);
                    float h = 0;
                    if ( !polys[0] )
                    {
                        TC_LOG_ERROR("general", "Pathfinding: Invalid poly ref on getPolyHeight");
                    }
                    dtStatus s = navQuery->getPolyHeight(polys[0], result, &h);
                   // printf("getPolyHeight: result[0] = %f result[1] = %f result[2] = %f , h = %f\n",result[0],result[1],result[2],h); 
                    if ( dtStatusFailed(s) )
                    {
                        //Sembra che in determinati punti fallisca , forse perché esattamente sulla connessione fra 2 poligoni
                        
                    }else{
                        result[1] = h;
                    }
                    dtVcopy(iterPos, result);
                    ;
                    // Handle end of path and off-mesh links when close enough.
                    if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f))
                    {
                        // Reached end of path.
                        iterPos[0] = targetPos[0];
                        iterPos[2] = targetPos[2];//La z del poligono navmesh non corrisponde con l'altezza
                        if (m_nsmoothPath < MAX_SMOOTH)
                        {
                            dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
                            m_nsmoothPath++;
                        }
                        break;
                        ;
                    }
                    else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f))
                    {
                        ;
                        // Reached off-mesh connection.
                        float startPos[3], endPos[3];

                        // Advance the path up to and over the off-mesh connection.
                        dtPolyRef prevRef = 0, polyRef = polys[0];
                        int npos = 0;
                        while (npos < npolys && polyRef != steerPosRef)
                        {
                            prevRef = polyRef;
                            polyRef = polys[npos];
                            npos++;
                        }
                        for (int i = npos; i < npolys; ++i)
                            polys[i-npos] = polys[i];
                        npolys -= npos;
                        ;
                        // Handle the connection.
                        if (m_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos))
                        {
                            if (m_nsmoothPath < MAX_SMOOTH)
                            {
                                dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
                                m_nsmoothPath++;
                                // Hack to make the dotted path not visible during off-mesh connection.
                                if (m_nsmoothPath & 1)
                                {
                                    dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
                                    m_nsmoothPath++;
                                }
                            }
                            // Move position at the other side of the off-mesh link.
                            dtVcopy(iterPos, endPos);
                            float h;
                            navQuery->getPolyHeight(polys[0], iterPos, &h);
                            iterPos[1] = h;
                        }
                        ;
                    }
                    ;
                    // Store results.
                    if (m_nsmoothPath < MAX_SMOOTH)
                    {
                        ;
                        dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
                        m_nsmoothPath++;
                        ;
                    }
                    ;
                }




                ;
                if (m_nsmoothPath < 2)
                {
                    TC_LOG_DEBUG("maps", "NavMesh corrotta o errore interno.");
                    result.result = PATHFIND_ERROR_NAVMESH;
                    result.path = NULL;
                    free(m_smoothPath);
                    delete polys;
                    delete navQuery;
                    free(mPathResults);
                    return result;
                }
                ;
                result.waypointcount = m_nsmoothPath;
                ;
                result.path = new float[3*(m_nsmoothPath)];
                ;
                for ( int c = 0; c < m_nsmoothPath; c++)
                {
                    result.path[c*3+0] = m_smoothPath[(c)*3+2];
                    result.path[c*3+1] = m_smoothPath[(c)*3+0];
                    result.path[c*3+2] = m_smoothPath[(c)*3+1];  //actualpath[c*3+1];
                    //
                    TC_LOG_DEBUG("maps", "Waypoint X=%f,Y=%f,Z=%f",result.path[c*3+0],result.path[c*3+1],result.path[c*3+2]);
                }
                TC_LOG_DEBUG("maps", "Tempo impiegato: %f",getcurrenttime()-tstart);
                free(m_smoothPath);
                delete polys;
                free(mPathResults);
                result.result = PATHFIND_OK;
                delete navQuery;
                return result;
            }
            else {
                TC_LOG_DEBUG("maps", "Impossibile trovare il poligono di partenza/fine : start=%llu, end=%llu",mStartRef,mEndRef);
                if (mStartRef == 0)
                {
                    result.result = PATHFIND_CANT_FIND_START_NAVMESH;
                }
                else {
                    result.result = PATHFIND_CANT_FIND_END_NAVMESH;
                }
                delete navQuery;
                return result;
            }
        }
    } else {
        TC_LOG_ERROR("general", "NavMesh non caricata");
        result.result = PATHFIND_NAVMESH_NOT_LOADED;
        result.path = new float[6];
        result.path[0] = srcx;
        result.path[1] = srcy;
        result.path[2] = srcz;
        result.path[3] = destx;
        result.path[4] = desty;
        result.path[5] = destz;
        result.waypointcount = 2;
    }
    delete navQuery;
    return result;
}

Position Map::getNextPositionOnPathToLocation(const float startx, const float starty, const float startz, const float endx, const float endy, const float endz)
{
    //convert to nav coords.
    float startPos[3]               = { starty, startz, startx };
    float endPos[3]                 = { endy, endz, endx };
    float mPolyPickingExtents[3]    = { 2.00f, 4.00f, 2.00f };
    dtQueryFilter* mPathFilter = new dtQueryFilter();
    int gx = 32 - (startx/533.333333f);
    int gy = 32 - (starty/533.333333f);
    Position pos = Position();
    pos.m_positionX = endx;
    pos.m_positionY = endy;
    pos.m_positionZ = endz;

    return pos;
}

dtMeshTile* Map::GetNavMeshTile(float x , float y)
{

    boost::mutex::scoped_lock lock(navmeshmutex);
    int gx=(int)(32-x/SIZE_OF_GRIDS);                       //grid x
    int gy=(int)(32-y/SIZE_OF_GRIDS);                       //grid y
    uint32 packedGridPos = packTileID(uint32(gx), uint32(gy));
    if(m_mmapTileMap.find(packedGridPos) == m_mmapTileMap.end())
        return NULL;

    uint32 packedTilePos = m_mmapTileMap[packedGridPos];
    uint32 tileX, tileY;
    unpackTileID(packedTilePos, tileX, tileY);
    const dtMeshTile* tile = m_navMesh->getTileAt(int(tileX), int(tileY),1);
    return (dtMeshTile*)tile;


}



void Map::UnloadNavMesh(int gx, int gy)
{
    boost::mutex::scoped_lock lock(navmeshmutex);
    int i_id = GetId();
    uint32 packedGridPos = packTileID(uint32(gx), uint32(gy));
    if(m_mmapTileMap.find(packedGridPos) == m_mmapTileMap.end())
    {
        TC_LOG_DEBUG("maps", "Impossibile fare l'unload della mmtile (%d,%d)\n",gx,gy);
        return;
    }

    uint32 packedTilePos = m_mmapTileMap[packedGridPos];
    uint32 tileX, tileY;
    unpackTileID(packedTilePos, tileX, tileY);

    // unload, and mark as non loaded

    if (!m_navMesh->removeTile(m_navMesh->getTileRefAt(int(tileX), int(tileY),1), 0, 0))
    {
        TC_LOG_ERROR("general", "Impossibile fare l'unload della mmtile (%d,%d):Fallito removeTile\n",gx,gy);
        return;

    }
    m_mmapTileMap.erase(packedGridPos);

    TC_LOG_DEBUG("maps", "Unloaded mmtile %03i[%02i,%02i] from %03i(%u)", i_id, gx, gy, i_id, GetInstanceId());
}


PathViewer::PathViewer(Unit * u)
{
    waypointmobs.clear();
    ownerguid = u->GetGUID();
    validpv.insert(std::pair<PathViewer*,int>(this,0));
}
void PathViewer::UpdatePath(std::vector< float > path)
{
    if (validpv.find(this) == validpv.end() )
        return;
    Unit * owner = ObjectAccessor::GetObjectInWorld(ownerguid,(Unit*)NULL);
    for ( t_mobwplist::iterator it = waypointmobs.begin(); it != waypointmobs.end(); it++ )
    {
        if ( !ObjectAccessor::GetCreature(*owner,*it) )
            it = waypointmobs.erase(it);

    }


    if ( path.size()/3 > waypointmobs.size() )
    {
        for ( int i = 0; i < path.size()/3-waypointmobs.size(); i++ )
        {
            if ( !Trinity::IsValidMapCoord(path[path.size()+i*3+0],path[path.size()+i*3+1],path[path.size()+i*3+2]) )
                continue;
            Creature* mob = owner->SummonCreature(1,path[path.size()+i*3+0],path[path.size()+i*3+1],path[path.size()+i*3+2]);
            mob->SetVisible(true);
            if (!mob)
                abort();
            waypointmobs.push_back(mob->GetGUID());

        }


    }
    if ( path.size()/3 < waypointmobs.size() )
    {
        for ( int i = 0; i < waypointmobs.size()-path.size()/3 && waypointmobs.size() > 0; i++ )
        {
            Creature * first = ObjectAccessor::GetCreature(*owner,waypointmobs.front());
            if ( first )
            {
                first->AddObjectToRemoveList();
            }
        }

    }
    int k = 0;

    for ( t_mobwplist::iterator it = waypointmobs.begin(); it != waypointmobs.end(); it++ )
    {
        Creature * mob = ObjectAccessor::GetCreature(*owner,*it);
        if ( mob )
        {
            mob->UpdatePosition(path[k*3+0],path[k*3+1],path[k*3+2],0.0,true);
            mob->StopMoving();
        }
        k++;
    }
}
PathViewer::~PathViewer()
{

    validpv.erase(this);

    for ( t_mobwplist::iterator it = waypointmobs.begin(); it != waypointmobs.end(); it++ )
    {
        Creature * mob = ObjectAccessor::GetObjectInWorld(*it,(Creature*)NULL);
        if ( mob )
        {
            mob->AddObjectToRemoveList();
        }

    }

}

