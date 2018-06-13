import importlib

from iawake.core.types.types import (
    List,
    ReturnType,
)


def load_module_member(member_path):
    module_path, member_name = member_path.rsplit('.', 1)
    module = importlib.import_module(module_path)
    return getattr(module, member_name)


def _validate_response_item(instance, return_type, item):
    if issubclass(return_type, ReturnType):
        assert return_type.validate_response(item), instance
    else:
        assert isinstance(item, return_type), instance


def validate_service_element_response(instance, response):
    assert instance.return_type != NotImplemented, instance
    if isinstance(instance.return_type, List):
        return_type = instance.return_type.return_type
        items = response
    else:
        return_type = instance.return_type
        items = [response]

    for item in items:
        _validate_response_item(instance, return_type, item)
