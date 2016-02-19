#ifndef EMPTY_H
#define EMPTY_H

#include "ocra/control/TaskManagers/TaskSequence.h"
#include "../sequenceTools.h"

// namespace sequence {

    class Empty : public ocra::TaskSequence
    {
        protected:
            virtual void doInit(ocra::Controller& ctrl, ocra::Model& model);
            virtual void doUpdate(double time, ocra::Model& state, void** args);
        private:
            // Full posture task
    };

// }


#endif
