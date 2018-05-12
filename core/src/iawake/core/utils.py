import importlib


def load_module_member(member_path):
    module_path, member_name = member_path.rsplit('.', 1)
    print module_path, member_name
    module = importlib.import_module(module_path)
    return getattr(module, member_name)
