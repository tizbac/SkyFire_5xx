/*
 * Copyright (C) 2008-2012 TrinityCore <http://www.trinitycore.org/>
 * Copyright (C) 2005-2009 MaNGOS <http://getmangos.com/>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "HomeMovementGeneratorPathFind.h"
#include "Creature.h"
#include "CreatureAI.h"
#include "WorldPacket.h"
#include "MoveSplineInit.h"
#include "MoveSpline.h"
#include "Pathfinding.h"
#include "ObjectAccessor.h"
HomeMovementGeneratorPathFind<Creature>::~HomeMovementGeneratorPathFind()
{
    Unit * u;
    u = ObjectAccessor::GetObjectInWorld ( guid, ( Unit* ) NULL );
    if ( u ) {
        boost::mutex::scoped_lock lock ( u->GetMap()->GetPathFindingMgr()->listsmutex );
        u->GetMap()->GetPathFindingMgr()->RemovePathfind ( GetPathFindingState() );
        if ( u->HasUnitState ( UNIT_STATE_EVADE ) )
            u->ClearUnitState ( UNIT_STATE_EVADE );


    }



}
void HomeMovementGeneratorPathFind<Creature>::Initialize ( Creature & owner )
{
    owner.AddUnitState ( UNIT_STATE_EVADE );
    guid = owner.GetGUID();
    _setTargetLocation ( owner );
}

void HomeMovementGeneratorPathFind<Creature>::Reset ( Creature & )
{
}

void HomeMovementGeneratorPathFind<Creature>::_setTargetLocation ( Creature & owner )
{
    if ( !&owner )
        return;

    if ( owner.HasUnitState ( UNIT_STATE_ROOT | UNIT_STATE_STUNNED | UNIT_STATE_DISTRACTED ) )
        return;

    float x, y, z, o;
    // at apply we can select more nice return points base at current movegen
    //if (owner.GetMotionMaster()->empty() || !owner.GetMotionMaster()->top()->GetResetPosition(owner,x,y,z))
    //{
    owner.GetHomePosition ( x, y, z, o );

    if ( !GetPathFindingState() || !owner.GetMap()->GetPathFindingMgr()->IsValid ( GetPathFindingState() ) )
        SetPathFindingState ( owner.GetMap()->GetPathFindingMgr()->AddPathfind ( ( Unit* ) &owner,x,y,z,owner.GetSpeed ( MOVE_RUN ) ) );
    else
        GetPathFindingState()->UpdateDestination ( x,y,z,0 );
    arrived = false;
    owner.ClearUnitState ( UNIT_STATE_ALL_STATE & ~UNIT_STATE_EVADE );
}

bool HomeMovementGeneratorPathFind<Creature>::Update ( Creature &owner, const uint32 /*time_diff*/ )
{
    owner.GetMap()->GetPathFindingMgr()->listsmutex.lock();
    
    if ( !GetPathFindingState() || !owner.GetMap()->GetPathFindingMgr()->IsValid ( GetPathFindingState() ) ) {
        _setTargetLocation ( owner );
    }
    float x,y,z,o;
    owner.GetHomePosition ( x,y,z,o );
    if ( GetPathFindingState() && GetPathFindingState()->HasArrived() ) {
        owner.AddUnitMovementFlag ( MOVEMENTFLAG_WALKING );

        // restore orientation of not moving creature at returning to home
        if ( owner.GetDefaultMovementType() == IDLE_MOTION_TYPE ) {
            //sLog->outDebug(LOG_FILTER_MAPS, "Entering HomeMovement::GetDestination(z,y,z)");
            
            owner.SetOrientation ( o );
            WorldPacket packet;
            owner.BuildHeartBeatMsg ( &packet );
            owner.SendMessageToSet ( &packet, false );
        }
        owner.GetMap()->GetPathFindingMgr()->listsmutex.unlock();

        arrived = true;
    } else if ( GetPathFindingState() && GetPathFindingState()->status == PATHFINDINGSTATUS_DEST_UNREACHABLE ) {
        if ( owner.GetDefaultMovementType() == IDLE_MOTION_TYPE ) {
            //sLog->outDebug(LOG_FILTER_MAPS, "Entering HomeMovement::GetDestination(z,y,z)");
            owner.SetOrientation ( o );
            WorldPacket packet;
            owner.BuildHeartBeatMsg ( &packet );
            owner.SendMessageToSet ( &packet, false );
        }
        owner.UpdatePosition ( owner.GetHomePosition(),true );
        owner.StopMoving();
        arrived = true;
    }
    owner.GetMap()->GetPathFindingMgr()->listsmutex.unlock();

    return !arrived;
}

void HomeMovementGeneratorPathFind<Creature>::Finalize ( Creature& owner )
{
    if ( arrived ) {
        owner.ClearUnitState ( UNIT_STATE_EVADE );
        owner.SetWalk ( true );
        owner.LoadCreaturesAddon ( true );
        owner.AI()->JustReachedHome();
    }
}
