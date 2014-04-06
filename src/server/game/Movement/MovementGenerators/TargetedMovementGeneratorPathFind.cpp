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

#include "ByteBuffer.h"
#include "TargetedMovementGeneratorPathFind.h"
#include "Errors.h"
#include "Creature.h"
#include "CreatureAI.h"
#include "World.h"
#include "MoveSplineInit.h"
#include "MoveSpline.h"

#include "Player.h"
#include "Pathfinding.h"
#include <cmath>

void PetTeleportToOwnerDelayed( uint64 petguid, uint64 ownerguid )
{
    Pet * pet = ObjectAccessor::GetObjectInWorld ( petguid, ( Pet* ) NULL );
    Unit * owner = ObjectAccessor::GetObjectInWorld ( ownerguid, ( Unit* ) NULL );
    if ( !pet || !owner )
        return;
    Position pos;
    owner->GetPosition(&pos);
    pet->SetPosition(pos);
}
void CreatureEvadeDelayed ( uint64 guid )
{
    Creature * c;
    c = ObjectAccessor::GetObjectInWorld ( guid, ( Creature* ) NULL );
    if ( c ) {
        if ( c->AI() )
            c->AI()->EnterEvadeMode();
    }
}

template <class T,typename D> TargetedMovementGeneratorMediumPathFind<T,D>::~TargetedMovementGeneratorMediumPathFind()
{
    Unit * u;
    u = ObjectAccessor::GetObjectInWorld ( guid, ( Unit* ) NULL );
    if ( u ) {
        boost::mutex::scoped_lock lock ( u->GetMap()->GetPathFindingMgr()->listsmutex );
        u->GetMap()->GetPathFindingMgr()->RemovePathfind ( GetPathFindingState() );
    }
}

template<class T, typename D>
void TargetedMovementGeneratorMediumPathFind<T,D>::_setTargetLocation ( T &owner )
{
    bool pfexisting = false;
    if ( owner.GetMap()->GetPathFindingMgr()->IsValid ( GetPathFindingState() ) ) {
        pfexisting = true;
        //Inizializza pathfinding
        //printf("Init pathfind\n");
        

    }
    if ( !i_target.isValid() || !i_target->IsInWorld() )
        return;

    if ( owner.HasUnitState ( UNIT_STATE_NOT_MOVE ) )
        return;

    float x, y, z;
    bool update_anyway = false;
   /* if ( Player * pltarg = i_target->ToPlayer() )
        update_anyway = pltarg->m_pred->HasMovementJustStarted();*/
    if ( i_offset && i_target->IsWithinDistInMap ( &owner,2*i_offset ) ) {
        if ( pfexisting && !GetPathFindingState()->arrived && !update_anyway)
            return;

        owner.GetPosition ( x, y, z );
    } else if ( !i_offset ) {
        if ( i_target->IsWithinMeleeRange ( &owner ) && !update_anyway)
            return;

        // to nearest random contact position
        i_target->GetRandomContactPoint ( &owner, x, y, z, 0, MELEE_RANGE - 0.5f );
    } else {
        if ( i_target->IsWithinDistInMap ( &owner, i_offset + 1.0f ) && !update_anyway )
            return;
        // to at i_offset distance from target and i_angle from target facing
        i_target->GetClosePoint ( x, y, z, owner.GetObjectSize(), i_offset, i_angle );
    }

    /*
        We MUST not check the distance difference and avoid setting the new location for smaller distances.
        By that we risk having far too many GetContactPoint() calls freezing the whole system.
        In TargetedMovementGenerator<T>::Update() we check the distance to the target and at
        some range we calculate a new position. The calculation takes some processor cycles due to vmaps.
        If the distance to the target it too large to ignore,
        but the distance to the new contact point is short enough to be ignored,
        we will calculate a new contact point each update loop, but will never move to it.
        The system will freeze.
        ralf

        //We don't update Mob Movement, if the difference between New destination and last destination is < BothObjectSize
        float  bothObjectSize = i_target->GetObjectBoundingRadius() + owner.GetObjectBoundingRadius() + CONTACT_DISTANCE;
        if( i_destinationHolder.HasDestination() && i_destinationHolder.GetDestinationDiff(x,y,z) < bothObjectSize )
            return;
    */


    D::_addUnitStateMove ( owner );
    i_targetReached = false;
    i_recalculateTravel = false;

    /*Movement::MoveSplineInit init ( owner );
    init.MoveTo ( x,y,z );
    init.SetWalk ( ( ( D* ) this )->EnableWalking() );
    init.Launch();*/
    if ( !pfexisting )
    {
        SetPathFindingState ( owner.GetMap()->GetPathFindingMgr()->AddPathfind ( &owner,x,y,z,owner.GetSpeed(MOVE_RUN) ) );
    }
    if ( owner.IsControlledByPlayer() )
    {
        GetPathFindingState()->petownerposition = TrinityVector3 ( i_target->GetPositionX(),i_target->GetPositionY(),i_target->GetPositionZ() );

    }else{
        GetPathFindingState()->petownerposition = TrinityVector3 (G3D::nan(),G3D::nan(),G3D::nan());
    }
    //printf("New destination %f %f %f\n",x,y,z);
    G3D::Vector3 latency_offset(0,0,0);
    /*if ( Player * pltarg = i_target->ToPlayer() )
    {
        latency_offset = pltarg->m_pred->EstimateClientSideOffset();
        
    }*/
    //std::cout << "Latency offset: " << latency_offset.toString() << std::endl;
    x += latency_offset.x;
    y += latency_offset.y;
    z += latency_offset.z;
   /* lastdestination.x = x;
    lastdestination.y = y;
    lastdestination.z = z;*/
    GetPathFindingState()->UpdateDestination ( x,y,z,i_target->GetUnitMovementFlags() );
}

template<>
void TargetedMovementGeneratorMediumPathFind<Player,ChaseMovementGeneratorPathFind<Player> >::UpdateFinalDistance ( float /*fDistance*/ )
{
    // nothing to do for Player
}

template<>
void TargetedMovementGeneratorMediumPathFind<Player,FollowMovementGeneratorPathFind<Player> >::UpdateFinalDistance ( float /*fDistance*/ )
{
    // nothing to do for Player
}

template<>
void TargetedMovementGeneratorMediumPathFind<Creature,ChaseMovementGeneratorPathFind<Creature> >::UpdateFinalDistance ( float fDistance )
{
    i_offset = fDistance;
    i_recalculateTravel = true;
}

template<>
void TargetedMovementGeneratorMediumPathFind<Creature,FollowMovementGeneratorPathFind<Creature> >::UpdateFinalDistance ( float fDistance )
{
    i_offset = fDistance;
    i_recalculateTravel = true;
}

template<class T, typename D>
PathFindingState * TargetedMovementGeneratorMediumPathFind<T,D>::GetPathFindingState()
{
  return PathFindingMovementGenerator::GetPathFindingStateX();

}
template<class T, typename D>
void TargetedMovementGeneratorMediumPathFind<T,D>::SetPathFindingState(PathFindingState* st)
{
  PathFindingMovementGenerator::SetPathFindingStateX(st);

}

template<class T, typename D>
bool TargetedMovementGeneratorMediumPathFind<T,D>::Update ( T &owner, const uint32 & time_diff )
{
    boost::mutex::scoped_lock lock ( owner.GetMap()->GetPathFindingMgr()->listsmutex );
    if ( !i_target.isValid() || !i_target->IsInWorld() )
        return false;
    bool pathfinding_started = true;
    if ( !owner.GetMap()->GetPathFindingMgr()->IsValid ( GetPathFindingState() ) ) {
        pathfinding_started = false;
    }
    if ( !owner.isAlive() )
        return true;

    if ( owner.HasUnitState ( UNIT_STATE_NOT_MOVE ) ) {
        D::_clearUnitStateMove ( owner );
        if ( pathfinding_started && !GetPathFindingState()->pause )
        {
            GetPathFindingState()->Pause();
        }
        return true;
    }

    // prevent movement while casting spells with cast time or channel time
    if ( owner.IsNonMeleeSpellCasted ( false, false,  true ) || owner.HasUnitState(UNIT_STATE_ROOT) || owner.HasUnitState(UNIT_STATE_STUNNED) || owner.HasUnitState(UNIT_STATE_FLEEING)) {
        if ( !owner.IsStopped() )
        {
            owner.StopMoving();
        }
        if ( pathfinding_started && !GetPathFindingState()->pause )
        {
            GetPathFindingState()->Pause();
        }
        return true;
    }

    // prevent crash after creature killed pet
    if ( static_cast<D*> ( this )->_lostTarget ( owner ) ) {
        D::_clearUnitStateMove ( owner );
        if ( pathfinding_started && !GetPathFindingState()->pause )
        {
            GetPathFindingState()->Pause();
        }
        return true;
    }
    if ( pathfinding_started && GetPathFindingState()->pause )
    {
        GetPathFindingState()->UnPause();
    }

    //G3D::Vector3 dest(GetPathFindingState()->destx,GetPathFindingState()->desty,GetPathFindingState()->destz);
    i_recheckDistance.Update ( time_diff );
    if ( i_recheckDistance.Passed() ) {
        i_recheckDistance.Reset ( 50 );
        //More distance let have better performance, less distance let have more sensitive reaction at target move.
        float allowed_dist = 0.3f;
        float dist = ( lastdestination - G3D::Vector3 ( i_target->GetPositionX(),i_target->GetPositionY(),i_target->GetPositionZ() ) ).squaredLength();
        sLog->outDebug(LOG_FILTER_MAPS,"PF: allowed_dist=%f dist=%f\n",allowed_dist*allowed_dist,dist);
        if ( dist >= allowed_dist * allowed_dist )
        {
            _setTargetLocation ( owner );
            lastdestination = G3D::Vector3 ( i_target->GetPositionX(),i_target->GetPositionY(),i_target->GetPositionZ() );
        }
    }

    if ( pathfinding_started && GetPathFindingState()->arrived ) { /* GetPathFindingState() è valido per forza dato che Update non è chiamato prima di initialize */
        static_cast<D*> ( this )->MovementInform ( owner );
        if ( i_angle == 0.f && !owner.HasInArc ( 0.01f, i_target.getTarget() ) )
            owner.SetInFront ( i_target.getTarget() );

        if ( !i_targetReached ) {
            i_targetReached = true;
            static_cast<D*> ( this )->_reachTarget ( owner );
        }
    } else {
        if ( i_recalculateTravel )
        {
            //printf("i_recalculateTravel\n");
            _setTargetLocation ( owner );
        }
    }

    if ( GetPathFindingState() ) {
        //sLog->outString("%p: paused=%s , rooted=%s",this,GetPathFindingState()->pause ? "true" : "false",owner.HasUnitState(UNIT_STAT_ROOT)? "true" : "false" );
        if ( owner.HasUnitState ( UNIT_STATE_ROOT |  UNIT_STATE_STUNNED | UNIT_STATE_DIED | UNIT_STATE_DISTRACTED | UNIT_STATE_CASTING | UNIT_STATE_CONFUSED ) ) {

            if ( ( !GetPathFindingState()->pause ) ) {
                GetPathFindingState()->Pause();
            }
            return true;
        } else {
            if ( GetPathFindingState()->pause ) {
                GetPathFindingState()->UnPause();
            }


        }
    }

    if ( unreachabletimer > 5000 ) {
        Creature * c = ( &owner )->ToCreature();
        if ( c ) {
            if ( c->IsControlledByPlayer() ) {
                Unit * owner = c->GetOwner();
                if ( owner )
                {
                    if ( i_target->GetGUID() == owner->GetGUID() )
                    {
                        c->GetMap()->mtcalls->post ( boost::bind ( &PetTeleportToOwnerDelayed,c->GetGUID(),owner->GetGUID() ) );
                        return true;
                        unreachabletimer = 0;
                    }
                    
                    
                }
            } else {
                if ( !c->GetMap()->IsDungeon() )
                {
                    if ( c->AI() && c->getVictim() == i_target.getTarget() )
                        c->GetMap()->mtcalls->post ( boost::bind ( &CreatureEvadeDelayed,c->GetGUID() ) );
                }
            }

        }
        return false;
    }
    /**
     * Per i pet nel caso ci sia la necessita di splittare il percorso, di sicuro è troppo lungo il percorso , quindi andrà teleportato doopo un certo tempo
     * 
     * */
    if ( (GetPathFindingState() && GetPathFindingState()->status == PATHFINDINGSTATUS_DEST_UNREACHABLE) || (GetPathFindingState() && GetPathFindingState()->status == PATHFINDINGSTATUS_PARTIAL && owner.IsControlledByPlayer() ) )
        unreachabletimer += time_diff;
    else
        unreachabletimer = 0;
    // prevent movement while casting spells with cast time or channel time

    if ( owner.HasUnitState ( UNIT_STATE_CASTING ) && GetPathFindingState() ) {
        if ( !owner.IsStopped() ) {

            GetPathFindingState()->Pause();

        }
        return true;
    }
    float calc_speed;
    calc_speed = owner.GetSpeed( MOVE_RUN);
    
    if ( owner.ToCreature() && ( owner.ToCreature()->GetOwnerGUID()  ) && i_target.isValid())
    {
        float player_speed = i_target->GetSpeed(MOVE_RUN);
        calc_speed = owner.m_speed_rate[MOVE_RUN]*player_speed*1.15;
        
    }
    if ( GetPathFindingState() ) {
        if ( GetPathFindingState()->speed != calc_speed) {
            if ( owner.GetSpeed ( MOVE_RUN ) > 0.0 )
                GetPathFindingState()->speed = calc_speed;
            else
                sLog->outError ( "TargetedMovementGenerator: Velocità non valida %f",owner.GetSpeed ( MOVE_RUN ) );
            GetPathFindingState()->mustrecalculate = true;
            //printf("Nuova velocità\n");
        }
    }


    return true;
}

//-----------------------------------------------//
template<class T>
void ChaseMovementGeneratorPathFind<T>::_reachTarget ( T &owner )
{
    if ( owner.IsWithinMeleeRange ( this->i_target.getTarget() ) )
        owner.Attack ( this->i_target.getTarget(),true );
}

template<>
void ChaseMovementGeneratorPathFind<Player>::Initialize ( Player &owner )
{
    owner.AddUnitState ( UNIT_STATE_CHASE|UNIT_STATE_CHASE_MOVE );
    {
        /*boost::mutex::scoped_lock lock ( owner.GetMap()->GetPathFindingMgr()->listsmutex );
        if ( !owner.GetMap()->GetPathFindingMgr()->IsValid ( GetPathFindingState() ) )
            SetPathFindingState ( owner.GetMap()->GetPathFindingMgr()->AddPathfind ( &owner,i_target->GetPositionX(),i_target->GetPositionY(),i_target->GetPositionZ(),owner.GetSpeed ( MOVE_RUN ) ) );*/
        _setTargetLocation ( owner );
    }
    guid = owner.GetGUID();

}

template<>
void ChaseMovementGeneratorPathFind<Creature>::Initialize ( Creature &owner )
{
    owner.SetWalk ( false );
    owner.AddUnitState ( UNIT_STATE_CHASE|UNIT_STATE_CHASE_MOVE );
    {
        boost::mutex::scoped_lock lock ( owner.GetMap()->GetPathFindingMgr()->listsmutex );
        /*if ( !owner.GetMap()->GetPathFindingMgr()->IsValid ( GetPathFindingState() ) )
        {
           /* if ( !i_target.isValid() || !i_target->IsInWorld() )
              sLog->outError("ChaseMovementGeneratorPathFind<Creature>::Initialize : Invalid target!");
            else
              SetPathFindingState ( owner.GetMap()->GetPathFindingMgr()->AddPathfind ( &owner,i_target->GetPositionX(),i_target->GetPositionY(),i_target->GetPositionZ(),owner.GetSpeed ( MOVE_RUN ) ) );
        */
            _setTargetLocation ( owner );
        //}
        
    }
    guid = owner.GetGUID();
}

template<class T>
void ChaseMovementGeneratorPathFind<T>::Finalize ( T &owner )
{
    owner.ClearUnitState ( UNIT_STATE_CHASE|UNIT_STATE_CHASE_MOVE );
}

template<class T>
void ChaseMovementGeneratorPathFind<T>::Reset ( T &owner )
{
    Initialize ( owner );
}

template<class T>
void ChaseMovementGeneratorPathFind<T>::MovementInform ( T & /*unit*/ )
{
}

template<>
void ChaseMovementGeneratorPathFind<Creature>::MovementInform ( Creature &unit )
{
    // Pass back the GUIDLow of the target. If it is pet's owner then PetAI will handle
    if ( unit.AI() )
        unit.AI()->MovementInform ( CHASE_MOTION_TYPE, i_target.getTarget()->GetGUIDLow() );
}

//-----------------------------------------------//
template<>
bool FollowMovementGeneratorPathFind<Creature>::EnableWalking() const
{
    return i_target.isValid() && i_target->IsWalking();
}

template<>
bool FollowMovementGeneratorPathFind<Player>::EnableWalking() const
{
    return false;
}

template<>
void FollowMovementGeneratorPathFind<Player>::_updateSpeed ( Player &/*u*/ )
{
    // nothing to do for Player
}

template<>
void FollowMovementGeneratorPathFind<Creature>::_updateSpeed ( Creature &u )
{
    u.UpdateSpeed ( MOVE_RUN,true );
    u.UpdateSpeed ( MOVE_WALK,true );
    u.UpdateSpeed ( MOVE_SWIM,true );

    float calc_speed;
    calc_speed = u.GetSpeed( MOVE_RUN);
    
    if ( u.ToCreature() && ( u.ToCreature()->GetOwnerGUID()  ) && i_target.isValid())
    {
        float player_speed = i_target->GetSpeed(MOVE_RUN);
        calc_speed = u.m_speed_rate[MOVE_RUN]*player_speed*1.15;
        
    }

    

    
    if ( GetPathFindingState() ) {
        if ( GetPathFindingState()->speed != calc_speed ) {
            if ( u.GetSpeed ( MOVE_RUN ) > 0.0 )
                GetPathFindingState()->speed = calc_speed;
            else
                sLog->outError ( "TargetedMovementGenerator: Velocità non valida %f",u.GetSpeed ( MOVE_RUN ) );
            GetPathFindingState()->mustrecalculate = true;
            //printf("Nuova velocità\n");
        }
    }
    if ( ! ( ( Creature& ) u ).isPet() || !i_target.isValid() || i_target->GetGUID() != u.GetOwnerGUID() )
        return;

}

template<>
void FollowMovementGeneratorPathFind<Player>::Initialize ( Player &owner )
{
    /*{
        boost::mutex::scoped_lock lock ( owner.GetMap()->GetPathFindingMgr()->listsmutex );
        if ( !owner.GetMap()->GetPathFindingMgr()->IsValid ( GetPathFindingState() ) )
            SetPathFindingState ( owner.GetMap()->GetPathFindingMgr()->AddPathfind ( &owner,G3D::nan(),G3D::nan(),G3D::nan(),owner.GetSpeed ( MOVE_RUN ) ) );

    }*/
    owner.AddUnitState ( UNIT_STATE_FOLLOW|UNIT_STATE_FOLLOW_MOVE );

    {
        boost::mutex::scoped_lock lock ( owner.GetMap()->GetPathFindingMgr()->listsmutex );
        _updateSpeed ( owner );
        _setTargetLocation ( owner );
    }
    guid = owner.GetGUID();
}

template<>
void FollowMovementGeneratorPathFind<Creature>::Initialize ( Creature &owner )
{
    /*{
        boost::mutex::scoped_lock lock ( owner.GetMap()->GetPathFindingMgr()->listsmutex );
        if ( !owner.GetMap()->GetPathFindingMgr()->IsValid ( GetPathFindingState() ) )
            SetPathFindingState ( owner.GetMap()->GetPathFindingMgr()->AddPathfind ( &owner,i_target->GetPositionX(),i_target->GetPositionY(),i_target->GetPositionZ(),owner.GetSpeed ( MOVE_RUN ) ) );

    }*/
    owner.AddUnitState ( UNIT_STATE_FOLLOW|UNIT_STATE_FOLLOW_MOVE );
    {
        boost::mutex::scoped_lock lock ( owner.GetMap()->GetPathFindingMgr()->listsmutex );
        
        _setTargetLocation ( owner );
        _updateSpeed ( owner );
    }
    guid = owner.GetGUID();
}

template<class T>
void FollowMovementGeneratorPathFind<T>::Finalize ( T &owner )
{
    owner.ClearUnitState ( UNIT_STATE_FOLLOW|UNIT_STATE_FOLLOW_MOVE );
    {
        boost::mutex::scoped_lock lock ( owner.GetMap()->GetPathFindingMgr()->listsmutex );
        _updateSpeed ( owner );
    }
}

template<class T>
void FollowMovementGeneratorPathFind<T>::Reset ( T &owner )
{
    Initialize ( owner );
}

template<class T>
void FollowMovementGeneratorPathFind<T>::MovementInform ( T & /*unit*/ )
{
}

template<>
void FollowMovementGeneratorPathFind<Creature>::MovementInform ( Creature &unit )
{
    // Pass back the GUIDLow of the target. If it is pet's owner then PetAI will handle
    if ( unit.AI() )
        unit.AI()->MovementInform ( FOLLOW_MOTION_TYPE, i_target.getTarget()->GetGUIDLow() );
}

//-----------------------------------------------//
template void TargetedMovementGeneratorMediumPathFind<Player,ChaseMovementGeneratorPathFind<Player> >::_setTargetLocation ( Player & );
template void TargetedMovementGeneratorMediumPathFind<Player,FollowMovementGeneratorPathFind<Player> >::_setTargetLocation ( Player & );
template void TargetedMovementGeneratorMediumPathFind<Creature,ChaseMovementGeneratorPathFind<Creature> >::_setTargetLocation ( Creature & );
template void TargetedMovementGeneratorMediumPathFind<Creature,FollowMovementGeneratorPathFind<Creature> >::_setTargetLocation ( Creature & );
template bool TargetedMovementGeneratorMediumPathFind<Player,ChaseMovementGeneratorPathFind<Player> >::Update ( Player &, const uint32 & );
template bool TargetedMovementGeneratorMediumPathFind<Player,FollowMovementGeneratorPathFind<Player> >::Update ( Player &, const uint32 & );
template bool TargetedMovementGeneratorMediumPathFind<Creature,ChaseMovementGeneratorPathFind<Creature> >::Update ( Creature &, const uint32 & );
template bool TargetedMovementGeneratorMediumPathFind<Creature,FollowMovementGeneratorPathFind<Creature> >::Update ( Creature &, const uint32 & );

template TargetedMovementGeneratorMediumPathFind<Player,ChaseMovementGeneratorPathFind<Player> >::~TargetedMovementGeneratorMediumPathFind();
template TargetedMovementGeneratorMediumPathFind<Player,FollowMovementGeneratorPathFind<Player> >::~TargetedMovementGeneratorMediumPathFind();
template TargetedMovementGeneratorMediumPathFind<Creature,ChaseMovementGeneratorPathFind<Creature> >::~TargetedMovementGeneratorMediumPathFind();
template TargetedMovementGeneratorMediumPathFind<Creature,FollowMovementGeneratorPathFind<Creature> >::~TargetedMovementGeneratorMediumPathFind();


template void ChaseMovementGeneratorPathFind<Player>::_reachTarget ( Player & );
template void ChaseMovementGeneratorPathFind<Creature>::_reachTarget ( Creature & );
template void ChaseMovementGeneratorPathFind<Player>::Finalize ( Player & );
template void ChaseMovementGeneratorPathFind<Creature>::Finalize ( Creature & );
template void ChaseMovementGeneratorPathFind<Player>::Reset ( Player & );
template void ChaseMovementGeneratorPathFind<Creature>::Reset ( Creature & );
template void ChaseMovementGeneratorPathFind<Player>::MovementInform ( Player &unit );

template void FollowMovementGeneratorPathFind<Player>::Finalize ( Player & );
template void FollowMovementGeneratorPathFind<Creature>::Finalize ( Creature & );
template void FollowMovementGeneratorPathFind<Player>::Reset ( Player & );
template void FollowMovementGeneratorPathFind<Creature>::Reset ( Creature & );
template void FollowMovementGeneratorPathFind<Player>::MovementInform ( Player &unit );

