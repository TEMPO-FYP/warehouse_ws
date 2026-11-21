#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

namespace b1z1
{

class DynamicAttachSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm)) {
      gzerr << "[DynamicAttachSystem] Must be attached to a valid model entity.\n";
      return;
    }

    if (_sdf->HasElement("parent_link")) {
      this->parentLinkName = _sdf->Get<std::string>("parent_link");
    }
    if (_sdf->HasElement("attach_topic")) {
      this->attachTopic = _sdf->Get<std::string>("attach_topic");
    }
    if (_sdf->HasElement("detach_topic")) {
      this->detachTopic = _sdf->Get<std::string>("detach_topic");
    }

    this->parentLinkEntity = this->model.LinkByName(_ecm, this->parentLinkName);
    this->gzNode.Subscribe(this->attachTopic, &DynamicAttachSystem::OnAttach, this);
    this->gzNode.Subscribe(this->detachTopic, &DynamicAttachSystem::OnDetach, this);

    gzmsg << "[DynamicAttachSystem] Configured for model '" << this->model.Name(_ecm)
          << "' parent_link='" << this->parentLinkName
          << "' attach='" << this->attachTopic
          << "' detach='" << this->detachTopic << "'\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager & _ecm) override
  {
    if (this->parentLinkEntity == gz::sim::kNullEntity) {
      this->parentLinkEntity = this->model.LinkByName(_ecm, this->parentLinkName);
      if (this->parentLinkEntity == gz::sim::kNullEntity) {
        return;
      }
    }

    bool doDetach = false;
    std::string attachReq;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      doDetach = this->pendingDetach;
      this->pendingDetach = false;
      if (!this->pendingAttach.empty()) {
        attachReq = this->pendingAttach;
        this->pendingAttach.clear();
      }
    }

    if ((doDetach || !attachReq.empty()) && this->jointEntity != gz::sim::kNullEntity) {
      _ecm.RequestRemoveEntity(this->jointEntity);
      gzmsg << "[DynamicAttachSystem] Detached '" << this->attachedTarget << "'.\n";
      this->jointEntity = gz::sim::kNullEntity;
      this->attachedTarget.clear();
    }

    if (attachReq.empty()) {
      return;
    }

    std::string childModelName = attachReq;
    std::string childLinkName;
    const auto sep = attachReq.find("::");
    if (sep != std::string::npos) {
      childModelName = attachReq.substr(0, sep);
      childLinkName = attachReq.substr(sep + 2);
    }

    const gz::sim::Entity childModelEntity = _ecm.EntityByComponents(
      gz::sim::components::Model(),
      gz::sim::components::Name(childModelName));
    if (childModelEntity == gz::sim::kNullEntity) {
      gzwarn << "[DynamicAttachSystem] Child model '" << childModelName << "' not found.\n";
      return;
    }

    gz::sim::Entity childLinkEntity = gz::sim::kNullEntity;
    gz::sim::Model childModel(childModelEntity);
    if (!childLinkName.empty()) {
      childLinkEntity = childModel.LinkByName(_ecm, childLinkName);
    } else {
      childLinkEntity = _ecm.EntityByComponents(
        gz::sim::components::Link(),
        gz::sim::components::ParentEntity(childModelEntity),
        gz::sim::components::CanonicalLink());
      if (childLinkEntity == gz::sim::kNullEntity) {
        auto links = childModel.Links(_ecm);
        if (!links.empty()) {
          childLinkEntity = links.front();
        }
      }
    }

    if (childLinkEntity == gz::sim::kNullEntity) {
      gzwarn << "[DynamicAttachSystem] No link found for child model '" << childModelName << "'.\n";
      return;
    }

    this->jointEntity = _ecm.CreateEntity();
    _ecm.CreateComponent(
      this->jointEntity,
      gz::sim::components::DetachableJoint({this->parentLinkEntity, childLinkEntity, "fixed"}));
    this->attachedTarget = attachReq;
    gzmsg << "[DynamicAttachSystem] Attached '" << attachReq
          << "' (link entity " << childLinkEntity << ") to '" << this->parentLinkName << "'.\n";
  }

private:
  void OnAttach(const gz::msgs::StringMsg & _msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pendingAttach = _msg.data();
    this->pendingDetach = false;
  }

  void OnDetach(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pendingDetach = true;
    this->pendingAttach.clear();
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  std::string parentLinkName{"b1z1_link06"};
  std::string attachTopic{"/b1z1/attach"};
  std::string detachTopic{"/b1z1/detach"};
  gz::sim::Entity parentLinkEntity{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity{gz::sim::kNullEntity};
  std::string attachedTarget;

  gz::transport::Node gzNode;
  std::mutex mutex;
  std::string pendingAttach;
  bool pendingDetach{false};
};

}  // namespace b1z1

GZ_ADD_PLUGIN(
  b1z1::DynamicAttachSystem,
  gz::sim::System,
  b1z1::DynamicAttachSystem::ISystemConfigure,
  b1z1::DynamicAttachSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(b1z1::DynamicAttachSystem, "b1z1::DynamicAttachSystem")
