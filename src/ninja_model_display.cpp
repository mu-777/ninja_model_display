//
// Created by ui-desktop on 15/12/14.
//

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <urdf/model.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/tf_link_updater.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"

#include "ninja_model_display/ninja_model_display.h"


namespace rviz {

    void linkUpdaterStatusFunction(StatusProperty::Level level,
                                   const std::string &link_name,
                                   const std::string &text,
                                   NinjaModelDisplay *display) {
        display->setStatus(level, QString::fromStdString(link_name), QString::fromStdString(text));
    }

    NinjaModelDisplay::NinjaModelDisplay()
            : has_new_transforms_(false), time_since_last_transform_(0.0f) {
        visual_enabled_property_ = new Property("Visual Enabled", true,
                                                "Whether to display the visual representation of the robot.",
                                                this, SLOT(updateVisualVisible()));

        collision_enabled_property_ = new Property("Collision Enabled", false,
                                                   "Whether to display the collision representation of the robot.",
                                                   this, SLOT(updateCollisionVisible()));

        update_rate_property_ = new FloatProperty("Update Interval", 0,
                                                  "Interval at which to update the links, in seconds. "
                                                          " 0 means to update every update cycle.",
                                                  this);
        update_rate_property_->setMin(0);

        alpha_property_ = new FloatProperty("Alpha", 1,
                                            "Amount of transparency to apply to the links.",
                                            this, SLOT(updateAlpha()));
        alpha_property_->setMin(0.0);
        alpha_property_->setMax(1.0);

        robot_description_property_ = new StringProperty("Robot Description", "robot_description",
                                                         "Name of the parameter to search for to load the robot description.",
                                                         this, SLOT(updateRobotDescription()));

        tf_prefix_property_ = new StringProperty("TF Prefix", "",
                                                 "Robot Model normally assumes the link name is the same as the tf frame name. "
                                                         " This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
                                                 this, SLOT(updateTfPrefix()));
    }

    NinjaModelDisplay::~NinjaModelDisplay() {
        if (initialized()) {
            delete robot_;
        }
    }

    void NinjaModelDisplay::onInitialize() {
        robot_ = new Robot(scene_node_, context_, "Robot: " + getName().toStdString(), this);

        updateVisualVisible();
        updateCollisionVisible();
        updateAlpha();
    }

    void NinjaModelDisplay::processMessage(const ninja_model_display::Alpha::ConstPtr &msg) {
        alpha_property_->setFloat(msg->alpha);
        updateAlpha();
    }

    void NinjaModelDisplay::updateAlpha() {
        robot_->setAlpha(alpha_property_->getFloat());
        context_->queueRender();
    }

    void NinjaModelDisplay::updateRobotDescription() {
        if (isEnabled()) {
            load();
            context_->queueRender();
        }
    }

    void NinjaModelDisplay::updateVisualVisible() {
        robot_->setVisualVisible(visual_enabled_property_->getValue().toBool());
        context_->queueRender();
    }

    void NinjaModelDisplay::updateCollisionVisible() {
        robot_->setCollisionVisible(collision_enabled_property_->getValue().toBool());
        context_->queueRender();
    }

    void NinjaModelDisplay::updateTfPrefix() {
        clearStatuses();
        context_->queueRender();
    }

    void NinjaModelDisplay::load() {
        std::string content;
        if (!update_nh_.getParam(robot_description_property_->getStdString(), content)) {
            std::string loc;
            if (update_nh_.searchParam(robot_description_property_->getStdString(), loc)) {
                update_nh_.getParam(loc, content);
            }
            else {
                clear();
                setStatus(StatusProperty::Error, "URDF",
                          "Parameter [" + robot_description_property_->getString()
                          + "] does not exist, and was not found by searchParam()");
                return;
            }
        }

        if (content.empty()) {
            clear();
            setStatus(StatusProperty::Error, "URDF", "URDF is empty");
            return;
        }

        if (content == robot_description_) {
            return;
        }

        robot_description_ = content;

        TiXmlDocument doc;
        doc.Parse(robot_description_.c_str());
        if (!doc.RootElement()) {
            clear();
            setStatus(StatusProperty::Error, "URDF", "URDF failed XML parse");
            return;
        }

        urdf::Model descr;
        if (!descr.initXml(doc.RootElement())) {
            clear();
            setStatus(StatusProperty::Error, "URDF", "URDF failed Model parse");
            return;
        }

        setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
        robot_->load(descr);
        robot_->update(TFLinkUpdater(context_->getFrameManager(),
                                     boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this),
                                     tf_prefix_property_->getStdString()));
    }

    void NinjaModelDisplay::onEnable() {
        load();
        robot_->setVisible(true);
    }

    void NinjaModelDisplay::onDisable() {
        robot_->setVisible(false);
        clear();
    }

    void NinjaModelDisplay::update(float wall_dt, float ros_dt) {
        time_since_last_transform_ += wall_dt;
        float rate = update_rate_property_->getFloat();
        bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

        if (has_new_transforms_ || update) {
            robot_->update(TFLinkUpdater(context_->getFrameManager(),
                                         boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this),
                                         tf_prefix_property_->getStdString()));
            context_->queueRender();

            has_new_transforms_ = false;
            time_since_last_transform_ = 0.0f;
        }
    }

    void NinjaModelDisplay::fixedFrameChanged() {
        has_new_transforms_ = true;
    }

    void NinjaModelDisplay::clear() {
        robot_->clear();
        clearStatuses();
        robot_description_.clear();
    }

    void NinjaModelDisplay::reset() {
        MessageFilterDisplay::reset();
        has_new_transforms_ = true;
    }

} // namespace rviz

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rviz::NinjaModelDisplay, rviz::Display
)

