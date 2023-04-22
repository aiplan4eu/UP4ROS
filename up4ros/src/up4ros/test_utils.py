from os.path import exists
import rospkg


def get_domain_and_problem(path_domain, path_problem):
    rospack = rospkg.RosPack()
    # needed to make the github pipelines work
    domain = (rospack.get_path('up4ros') + '/tests/' + path_domain)
    problem = (rospack.get_path('up4ros') + '/tests/' + path_problem)

    if not exists(domain):
        domain = (rospack.get_path('up4ros') + '/' + path_domain)
        problem = (rospack.get_path('up4ros') + '/' + path_problem)

    return domain, problem
