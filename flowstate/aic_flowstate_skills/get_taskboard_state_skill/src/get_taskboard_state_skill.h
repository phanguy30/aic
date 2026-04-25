#ifndef GET_TASKBOARD_STATE_SKILL_H_
#define GET_TASKBOARD_STATE_SKILL_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

class GetTaskboardStateSkill final : public intrinsic::skills::SkillInterface {
 public:
  static std::unique_ptr<intrinsic::skills::SkillInterface> CreateSkill();

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Preview(
      const intrinsic::skills::PreviewRequest& request,
      intrinsic::skills::PreviewContext& context) override;

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Execute(
      const intrinsic::skills::ExecuteRequest& request,
      intrinsic::skills::ExecuteContext& context) override;
};

#endif  // GET_TASKBOARD_STATE_SKILL_H_
