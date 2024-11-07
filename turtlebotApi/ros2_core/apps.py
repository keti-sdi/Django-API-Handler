from django.apps import AppConfig


class Ros2CoreConfig(AppConfig):
    default_auto_field = 'django.db.models.BigAutoField'
    name = 'ros2_core'

    def ready(self):
        from .ros2_executor import start_executor  
        start_executor()
