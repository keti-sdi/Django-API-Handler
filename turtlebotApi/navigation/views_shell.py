import json
import subprocess
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from django.utils.decorators import method_decorator
from django.views import View
import logging

logger = logging.getLogger(__name__)
@method_decorator(csrf_exempt, name='dispatch')
class GoalAPIView(View):

    def get(self, request, *args, **kwargs):
        return self.execute_goal(request)

    def post(self, request, *args, **kwargs):
        return self.execute_goal(request)

    def execute_goal(self, request):
        try:
            body = json.loads(request.body)
            x = body.get('x')
            y = body.get('y')

            if x is None or y is None:
                return JsonResponse({'error': 'Invalid coordinates'}, status=400)

       #     ssh_command = f"ssh root@10.0.5.53 'python3 /opt/ros/foxy/share/turtlebot3_navigation2/launch/goal.py --x {x} --y {y}'"
            ssh_command = f"ssh -o StrictHostKeyChecking=no root@10.0.5.53 'python3 /opt/ros/foxy/share/turtlebot3_navigation2/launch/goal.py --x {x} --y {y}'"

            process = subprocess.Popen(ssh_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            stdout, stderr = process.communicate()

            if process.returncode == 0:
                return JsonResponse({'message': 'TurtleBot moved successfully', 'output': stdout.decode()})
            else:
                return JsonResponse({'error': 'Failed to move TurtleBot', 'details': stderr.decode(),'details': stderr.decode(),'command': ssh_command}, status=500)

        except json.JSONDecodeError:
            return JsonResponse({'error': 'Invalid JSON'}, status=400)
        except Exception as e:
            return JsonResponse({'error': str(e)}, status=500)
