# battery/views.py
import logging
from django.http import JsonResponse
from django.views import View

logger = logging.getLogger('navigation')

class BatteryStateView(View):
    def get(self, request, *args, **kwargs):
        from ros2_core.ros2_executor import get_battery_state 
        battery_state = get_battery_state()
        if battery_state['voltage'] is None or battery_state['percentage'] is None:
            return JsonResponse({'error': 'Battery state not available'}, status=503)
        return JsonResponse(battery_state)
