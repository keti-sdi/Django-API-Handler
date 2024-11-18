# location/views.py
from django.http import JsonResponse
from django.views import View

class StartPosePublishingView(View):
    def get(self, request, *args, **kwargs):
        from ros2_core.ros2_executor import start_executor, pose_node
        start_executor()
        pose_node.start_publishing()
        return JsonResponse({'status': 'Pose publishing started'})

class StopPosePublishingView(View):
    def get(self, request, *args, **kwargs):
        return JsonResponse({'status': 'Pose publishing stopped'})

class TurtlebotPoseView(View):
    def get(self, request, *args, **kwargs):
        from ros2_core.ros2_executor import get_pose
        pose = get_pose()
        if pose['x'] is None or pose['y'] is None or pose['z'] is None:
            return JsonResponse({'error': 'Pose information not available'}, status=503)
        return JsonResponse(pose)
