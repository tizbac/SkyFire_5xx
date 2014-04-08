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

#include "PointMovementGeneratorPathFind.h"
#include "Errors.h"
#include "Creature.h"
#include "CreatureAI.h"
#include "World.h"
#include "MoveSplineInit.h"
#include "MoveSpline.h"
#include "Player.h"
#include "ObjectAccessor.h"
#include "Pathfinding.h"

namespace Movement
{
    UnitMoveType SelectSpeedType(uint32 moveFlags);
}

template<class T>
PointMovementGeneratorPathFind<T>::~PointMovementGeneratorPathFind()
{
    Unit * u;
    u = ObjectAccessor::GetObjectInWorld ( guid, ( Unit* ) NULL );
    if ( u ) {
        boost::mutex::scoped_lock lock ( u->GetMap()->GetPathFindingMgr()->listsmutex );
        u->GetMap()->GetPathFindingMgr()->RemovePathfind ( GetPathFindingState() );

    }
}
//----- Point Movement Generator
template<class T>
void PointMovementGeneratorPathFind<T>::DoInitialize ( T* unit )
{
    if ( !unit->IsStopped() )
        unit->StopMoving();

    unit->AddUnitState ( UNIT_STATE_ROAMING|UNIT_STATE_ROAMING_MOVE );
    boost::mutex::scoped_lock lock(unit->GetMap()->GetPathFindingMgr()->listsmutex);
    if (!GetPathFindingState() || !unit->GetMap()->GetPathFindingMgr()->IsValid(GetPathFindingState()))
    {
      SetPathFindingState(unit->GetMap()->GetPathFindingMgr()->AddPathfind(unit,i_x,i_y,i_z,speed > 0.01 ? speed : unit->GetSpeed(Movement::SelectSpeedType(unit->GetUnitMovementFlags())),true));
      GetPathFindingState()->isCharge = m_ischarge;
    }
    guid = unit->GetGUID();
}
template <class T> PathFindingState* PointMovementGeneratorPathFind<T>::GetPathFindingState()
{
  return MovementGeneratorMedium< T, PointMovementGeneratorPathFind<T> >::GetPathFindingStateX();
  
}
template <class T> void PointMovementGeneratorPathFind<T>::SetPathFindingState(PathFindingState* st)
{
  MovementGeneratorMedium< T, PointMovementGeneratorPathFind<T> >::SetPathFindingStateX(st);
          
}



template<class T>
bool PointMovementGeneratorPathFind<T>::DoUpdate ( T* unit, uint32 /*diff*/ )
{
    if ( !unit )
        return false;

    if ( unit->HasUnitState ( UNIT_STATE_ROOT | UNIT_STATE_STUNNED ) ) {
        unit->ClearUnitState ( UNIT_STATE_ROAMING_MOVE );
        if (unit->HasUnitState(UNIT_STATE_CHARGING))
            return false;
        else
        {
            //printf("PointMovement: Stunned and no charge\n");
            return true;
        }
    }
    
    unit->AddUnitState ( UNIT_STATE_ROAMING_MOVE );
    boost::mutex::scoped_lock lock(unit->GetMap()->GetPathFindingMgr()->listsmutex);
    if ( unit->GetMap()->GetPathFindingMgr()->IsValid(GetPathFindingState()) )
    {
      if ( GetPathFindingState()->status == PATHFINDINGSTATUS_DEST_UNREACHABLE )
          {
                Player * pl = unit->ToPlayer();
                if ( pl )
                {
                  pl->GetSession()->SendNotification("Target non raggiungibile");
                }
                return false;
          }
      if ( GetPathFindingState()->HasArrived() )
      {
        TC_LOG_DEBUG("movement", "PointMovement: Arrived.\n");
        unit->ClearUnitState(UNIT_STATE_MOVE);
        arrived = true;
        return false;
      }
      if ( !speed )
      {
          
        float calc_speed;
        calc_speed = unit->GetSpeed(Movement::SelectSpeedType(unit->GetUnitMovementFlags()));
        if ( GetPathFindingState() ) {
            if ( GetPathFindingState()->speed != calc_speed) {
                
                GetPathFindingState()->speed = calc_speed;
                GetPathFindingState()->mustrecalculate = true;
                //printf("Nuova velocità\n");
            }
        }
      }
    }
    return !arrived;
}

template<class T>
void PointMovementGeneratorPathFind<T>::DoFinalize ( T* unit )
{
    unit->ClearUnitState ( UNIT_STATE_ROAMING|UNIT_STATE_ROAMING_MOVE );

    if ( arrived )
        MovementInform ( unit );
}

template<class T>
void PointMovementGeneratorPathFind<T>::DoReset ( T* unit )
{
    if ( !unit->IsStopped() )
        unit->StopMoving();

    unit->AddUnitState ( UNIT_STATE_ROAMING|UNIT_STATE_ROAMING_MOVE );
}

template<class T>
void PointMovementGeneratorPathFind<T>::MovementInform ( T* /*unit*/ )
{
}

template <> void PointMovementGeneratorPathFind<Creature>::MovementInform ( Creature* unit )
{
    //if (id == EVENT_FALL_GROUND)
    //{
    //    unit.setDeathState(JUST_DIED);
    //    unit.SetFlying(true);
    //}
    if ( unit->AI() )
        unit->AI()->MovementInform ( POINT_MOTION_TYPE, id );
}

template void PointMovementGeneratorPathFind<Player>::DoInitialize ( Player* );
template void PointMovementGeneratorPathFind<Creature>::DoInitialize ( Creature* );
template void PointMovementGeneratorPathFind<Player>::DoFinalize ( Player* );
template void PointMovementGeneratorPathFind<Creature>::DoFinalize ( Creature* );
template void PointMovementGeneratorPathFind<Player>::DoReset ( Player* );
template void PointMovementGeneratorPathFind<Creature>::DoReset ( Creature* );
template bool PointMovementGeneratorPathFind<Player>::DoUpdate ( Player*, uint32 );
template bool PointMovementGeneratorPathFind<Creature>::DoUpdate ( Creature*, uint32 );
template PointMovementGeneratorPathFind<Player>::~PointMovementGeneratorPathFind();
template PointMovementGeneratorPathFind<Creature>::~PointMovementGeneratorPathFind();
template PathFindingState* PointMovementGeneratorPathFind<Player>::GetPathFindingState();
template PathFindingState* PointMovementGeneratorPathFind<Creature>::GetPathFindingState();

/*void AssistanceMovementGenerator::Finalize ( Unit &unit )
{
    unit.ToCreature()->SetNoCallAssistance ( false );
    unit.ToCreature()->CallAssistance();
    if ( unit.isAlive() )
        unit.GetMotionMaster()->MoveSeekAssistanceDistract ( sWorld->getIntConfig ( CONFIG_CREATURE_FAMILY_ASSISTANCE_DELAY ) );
}

bool EffectMovementGenerator::Update ( Unit &unit, const uint32 )
{
    PathFindingState * pfstate = unit.GetMotionMaster()->GetMotionSlot(MOTION_SLOT_ACTIVE)->GetPathFindingState();
    boost::mutex::scoped_lock lock(unit.GetMap()->GetPathFindingMgr()->listsmutex);
    if ( unit.GetMap()->GetPathFindingMgr()->IsValid(pfstate) )
    {
        return !pfstate->arrived;
    }
    return false;
}

void EffectMovementGenerator::Finalize ( Unit &unit )
{
    if ( unit.GetTypeId() != TYPEID_UNIT )
        return;
    PathFindingState * pfstate = unit.GetMotionMaster()->GetMotionSlot(MOTION_SLOT_ACTIVE)->GetPathFindingState();
    boost::mutex::scoped_lock lock(unit.GetMap()->GetPathFindingMgr()->listsmutex);
    if ( unit.GetMap()->GetPathFindingMgr()->IsValid(pfstate) )
    {
        if ( ( ( Creature& ) unit ).AI() && pfstate->arrived )
            ( ( Creature& ) unit ).AI()->MovementInform ( EFFECT_MOTION_TYPE, m_Id );
    }
    // Need restore previous movement since we have no proper states system
    //if (unit.isAlive() && !unit.HasUnitState(UNIT_STATE_CONFUSED|UNIT_STATE_FLEEING))
    //{
    //    if (Unit * victim = unit.getVictim())
    //        unit.GetMotionMaster()->MoveChase(victim);
    //    else
    //        unit.GetMotionMaster()->Initialize();
    //}
}*/
