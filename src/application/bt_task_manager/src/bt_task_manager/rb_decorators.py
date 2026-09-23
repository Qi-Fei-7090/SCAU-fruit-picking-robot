from py_trees import decorators
from py_trees import blackboard
from py_trees import common


class LoopWithCondition(decorators.Decorator):
    def __init__(self, name, child, condition):
        super(LoopWithCondition, self).__init__(name=name, child=child)
        self.success = 0
        self.condition = condition
        self.blackboard = blackboard.Blackboard()

    def initialise(self):
        self.success = 0

    def update(self):
        if self.decorated.status == common.Status.FAILURE:
            self.feedback_message = "failed, aborting [status: {}]".format(self.success)
            return common.Status.FAILURE
        elif self.decorated.status == common.Status.SUCCESS:
            self.success += 1
            self.feedback_message = "success [status: {}]".format(self.success)
            condition = self.blackboard.get(self.condition)
            if condition:
                return common.Status.RUNNING
            else:
                return common.Status.SUCCESS
        else:
            self.feedback_message = "running [status: {}]".format(self.success)
            return common.Status.RUNNING


class EndWithCondition(decorators.Decorator):
    def __init__(self, name, child, condition):
        super().__init__(name=name, child=child)
        self.success = 0
        self.condition = condition
        self.blackboard = blackboard.Blackboard()

    def initialise(self):
        self.success = 0

    def update(self):
        if self.decorated.status == common.Status.FAILURE:
            self.feedback_message = "failed, aborting [status: {}]".format(self.success)
            return common.Status.FAILURE
        elif self.decorated.status == common.Status.SUCCESS:
            self.success += 1
            self.feedback_message = "success [status: {}]".format(self.success)
            condition = self.blackboard.get(self.condition)
            if condition:
                return common.Status.RUNNING
            else:
                return common.Status.SUCCESS
        else:
            self.feedback_message = "running [status: {}]".format(self.success)
            return common.Status.RUNNING
