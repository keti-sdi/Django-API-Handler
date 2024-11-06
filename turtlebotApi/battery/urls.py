from django.urls import path
from .views import BatteryStateView
urlpatterns = [
    path('now-state/', BatteryStateView.as_view(), name='battery-state')
]
