/*
 * Copyright (C) 2011-2014 Project SkyFire <http://www.projectskyfire.org/>
 * Copyright (C) 2008-2014 TrinityCore <http://www.trinitycore.org/>
 * Copyright (C) 2005-2014 MaNGOS <http://getmangos.com/>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 3 of the License, or (at your
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

#ifndef TRINITY_MOVEMENTGENERATOR_H
#define TRINITY_MOVEMENTGENERATOR_H

#include "Define.h"
#include <ace/Singleton.h>
#include "ObjectRegistry.h"
#include "FactoryHolder.h"
#include "Common.h"
#include "MotionMaster.h"

class Unit;
class PathFindingState;
class PathFindingMovementGenerator
{
public:
  PathFindingMovementGenerator(){ pfstate = 0; }
  PathFindingState * GetPathFindingStateX()
  {
    return pfstate;
  }
  void SetPathFindingStateX(PathFindingState * pf)
  {
    pfstate = pf;
  }
private:
  PathFindingState* pfstate;
  
};



class MovementGenerator
{
    public:
        virtual ~MovementGenerator();

        virtual void DoInitialize(Unit*) = 0;
        virtual void DoFinalize(Unit*) = 0;

        virtual void DoReset(Unit*) = 0;

        virtual bool DoUpdate(Unit*, uint32 time_diff) = 0;

        virtual MovementGeneratorType GetMovementGeneratorType() = 0;

        virtual void unitSpeedChanged() { }

        // used by Evade code for select point to evade with expected restart default movement
        virtual bool GetResetPosition(Unit*, float& /*x*/, float& /*y*/, float& /*z*/) { return false; }
        
        virtual PathFindingState * GetPathFindingState(){return NULL;}
        virtual void SetPathFindingState(PathFindingState * st){}

};

template<class T, class D>
class MovementGeneratorMedium : public MovementGenerator , public PathFindingMovementGenerator

{
    public:
        void DoInitialize(Unit* u)
        {
            //u->AssertIsType<T>();
            (static_cast<D*>(this))->DoInitialize(static_cast<T*>(u));
        }

        void DoFinalize(Unit* u)
        {
            //u->AssertIsType<T>();
            (static_cast<D*>(this))->DoFinalize(static_cast<T*>(u));
        }

        void DoReset(Unit* u)
        {
            //u->AssertIsType<T>();
            (static_cast<D*>(this))->DoReset(static_cast<T*>(u));
        }

        bool DoUpdate(Unit* u, uint32 time_diff)
        {
            //u->AssertIsType<T>();
            return (static_cast<D*>(this))->DoUpdate(static_cast<T*>(u), time_diff);
        }
};

struct SelectableMovement : public FactoryHolder<MovementGenerator, MovementGeneratorType>
{
    SelectableMovement(MovementGeneratorType mgt) : FactoryHolder<MovementGenerator, MovementGeneratorType>(mgt) { }
};

template<class REAL_MOVEMENT>
struct MovementGeneratorFactory : public SelectableMovement
{
    MovementGeneratorFactory(MovementGeneratorType mgt) : SelectableMovement(mgt) { }

    MovementGenerator* Create(void *) const;
};

typedef FactoryHolder<MovementGenerator, MovementGeneratorType> MovementGeneratorCreator;
typedef FactoryHolder<MovementGenerator, MovementGeneratorType>::FactoryHolderRegistry MovementGeneratorRegistry;
typedef FactoryHolder<MovementGenerator, MovementGeneratorType>::FactoryHolderRepository MovementGeneratorRepository;
#endif
