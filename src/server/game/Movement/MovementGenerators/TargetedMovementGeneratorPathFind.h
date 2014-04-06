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

#ifndef TRINITY_TARGETEDMOVEMENTGENERATORPF_H
#define TRINITY_TARGETEDMOVEMENTGENERATORPF_H

#include "MovementGenerator.h"
#include "FollowerReference.h"
#include "TargetedMovementGenerator.h"
#include "Timer.h"
#include "Unit.h"
#include "ObjectAccessor.h"
class PathFindingState;
template<class T, typename D>
class TargetedMovementGeneratorMediumPathFind
: public MovementGeneratorMedium< T, D >, public TargetedMovementGeneratorBase
{
    protected:
        TargetedMovementGeneratorMediumPathFind(Unit &target, float offset, float angle) :
            TargetedMovementGeneratorBase(target), i_recheckDistance(0),
            i_offset(offset), i_angle(angle),
            i_recalculateTravel(false), i_targetReached(false), unreachabletimer(0), guid(0)
        {
        }
        ~TargetedMovementGeneratorMediumPathFind();

    public:
        bool DoUpdate(T &, const uint32 &);
        Unit* GetTarget() const { return i_target.getTarget(); }

        void unitSpeedChanged() { i_recalculateTravel=true; }
        void UpdateFinalDistance(float fDistance);
        PathFindingState* GetPathFindingState();
        void SetPathFindingState(PathFindingState* st);
    protected:
        void _setTargetLocation(T &);

        TimeTrackerSmall i_recheckDistance;
        float i_offset;
        float i_angle;
        bool i_recalculateTravel : 1;
        bool i_targetReached : 1;
        uint32 unreachabletimer;
        uint64 guid;
        G3D::Vector3 lastdestination;
};

template<class T>
class ChaseMovementGeneratorPathFind : public TargetedMovementGeneratorMediumPathFind<T, ChaseMovementGeneratorPathFind<T> >
{
    public:
        ChaseMovementGeneratorPathFind(Unit &target)
            : TargetedMovementGeneratorMediumPathFind<T, ChaseMovementGeneratorPathFind<T> >(target) {}
        ChaseMovementGeneratorPathFind(Unit &target, float offset, float angle)
            : TargetedMovementGeneratorMediumPathFind<T, ChaseMovementGeneratorPathFind<T> >(target, offset, angle) {}
        ~ChaseMovementGeneratorPathFind() {}

        MovementGeneratorType GetMovementGeneratorType() { return CHASE_MOTION_TYPE; }

        void Initialize(T &);
        void Finalize(T &);
        void Reset(T &);
        void MovementInform(T &);

        static void _clearUnitStateMove(T &u) { u.ClearUnitState(UNIT_STATE_CHASE_MOVE); }
        static void _addUnitStateMove(T &u)  { u.AddUnitState(UNIT_STATE_CHASE_MOVE); }
        bool EnableWalking() const { return false;}
        bool _lostTarget(T &u) const { return u.getVictim() != this->GetTarget(); }
        void _reachTarget(T &);
};

template<class T>
class FollowMovementGeneratorPathFind : public TargetedMovementGeneratorMediumPathFind<T, FollowMovementGeneratorPathFind<T> >
{
    public:
        FollowMovementGeneratorPathFind(Unit &target)
            : TargetedMovementGeneratorMediumPathFind<T, FollowMovementGeneratorPathFind<T> >(target){}
        FollowMovementGeneratorPathFind(Unit &target, float offset, float angle)
            : TargetedMovementGeneratorMediumPathFind<T, FollowMovementGeneratorPathFind<T> >(target, offset, angle) {}
        ~FollowMovementGeneratorPathFind() {}

        MovementGeneratorType GetMovementGeneratorType() { return FOLLOW_MOTION_TYPE; }

        void Initialize(T &);
        void Finalize(T &);
        void Reset(T &);
        void MovementInform(T &);

        static void _clearUnitStateMove(T &u) { u.ClearUnitState(UNIT_STATE_FOLLOW_MOVE); }
        static void _addUnitStateMove(T &u)  { u.AddUnitState(UNIT_STATE_FOLLOW_MOVE); }
        bool EnableWalking() const;
        bool _lostTarget(T &) const { return false; }
        void _reachTarget(T &) {}
    private:
        void _updateSpeed(T &u);
};

#endif

