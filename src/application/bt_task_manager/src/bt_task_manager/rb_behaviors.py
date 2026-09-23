from py_trees import behaviour
from py_trees import common
from py_trees.common import Status
from py_trees.behaviour import Behaviour


class ReturnStatus(behaviour.Behaviour):
    def __init__(self, name="ReturnStatus", status=common.Status.SUCCESS):
        super(ReturnStatus, self).__init__(name)
        self.return_status = status

    def setup(self, timeout):
        return True

    def initialise(self):
        return

    def update(self):
        return self.return_status

    def terminate(self, new_status):
        return


class SuccessEveryN(behaviour.Behaviour):
    """
    This behaviour updates it's status with :data:`~py_trees.common.Status.SUCCESS`
    once every N ticks, :data:`~py_trees.common.Status.FAILURE` otherwise.

    Args:
        name (:obj:`str`): name of the behaviour
        n (:obj:`int`): trigger success on every n'th tick

    .. tip::
       Use with decorators to change the status value as desired, e.g.
       :meth:`py_trees.meta.failure_is_running`
    """

    def __init__(self, name, n):
        super(SuccessEveryN, self).__init__(name)
        self.count = 0
        self.every_n = n

    def update(self):
        self.count += 1
        self.logger.debug("%s.update()][%s]" % (self.__class__.__name__, self.count))
        if self.count % self.every_n == 0:
            self.feedback_message = "now"
            return Status.SUCCESS
        else:
            self.feedback_message = "not yet"
            return Status.RUNNING


class FailureEveryN(behaviour.Behaviour):
    """
    This behaviour updates it's status with :data:`~py_trees.common.Status.SUCCESS`
    once every N ticks, :data:`~py_trees.common.Status.FAILURE` otherwise.

    Args:
        name (:obj:`str`): name of the behaviour
        n (:obj:`int`): trigger success on every n'th tick

    .. tip::
       Use with decorators to change the status value as desired, e.g.
       :meth:`py_trees.meta.failure_is_running`
    """

    def __init__(self, name, n):
        super(FailureEveryN, self).__init__(name)
        self.count = 0
        self.every_n = n

    def update(self):
        self.count += 1
        self.logger.debug("%s.update()][%s]" % (self.__class__.__name__, self.count))
        if self.count % self.every_n == 0:
            self.feedback_message = "now"
            return Status.FAILURE
        else:
            self.feedback_message = "not yet"
            return Status.RUNNING
