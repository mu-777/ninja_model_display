//
// Created by ui-desktop on 15/12/14.
//

#ifndef NINJA_MODEL_DISPLAY_NINJA_MODEL_DISPLAY_H
#define NINJA_MODEL_DISPLAY_NINJA_MODEL_DISPLAY_H

#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.h>
#include "std_msgs/Float32.h"
#include "ninja_model_display/Alpha.h"


#include <OgreVector3.h>

#include <map>

namespace Ogre {
    class Entity;

    class SceneNode;
}

namespace rviz {
    class Axes;
}

namespace rviz {

    class FloatProperty;

    class Property;

    class Robot;

    class StringProperty;

    class NinjaModelDisplay : public MessageFilterDisplay<ninja_model_display::Alpha> {
    Q_OBJECT

    public:
        NinjaModelDisplay();

        virtual ~NinjaModelDisplay();

        // Overrides from Display
        virtual void onInitialize();

        virtual void update(float wall_dt, float ros_dt);

        virtual void fixedFrameChanged();

        virtual void reset();

        void clear();

    private :
        void processMessage(const ninja_model_display::Alpha::ConstPtr &msg);

    private Q_SLOTS:

        void updateVisualVisible();

        void updateCollisionVisible();

        void updateTfPrefix();

        void updateAlpha();

        void updateRobotDescription();

    protected:
        /** @brief Loads a URDF from the ros-param named by our
         * "Robot Description" property, iterates through the links, and
         * loads any necessary models. */
        virtual void load();

        // overrides from Display
        virtual void onEnable();

        virtual void onDisable();

        Robot *robot_;                 ///< Handles actually drawing the robot

        bool has_new_transforms_;      ///< Callback sets this to tell our update function it needs to update the transforms

        float time_since_last_transform_;

        std::string robot_description_;

        Property *visual_enabled_property_;
        Property *collision_enabled_property_;
        FloatProperty *update_rate_property_;
        StringProperty *robot_description_property_;
        FloatProperty *alpha_property_;
        StringProperty *tf_prefix_property_;
    };

} // namespace rviz

#endif //NINJA_MODEL_DISPLAY_NINJA_MODEL_DISPLAY_H


