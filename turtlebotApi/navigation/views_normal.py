# navigation/views.py
from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
import logging

logger = logging.getLogger(__name__)

class GoalAPIView(APIView):
    def post(self, request):
        logger.info("Received goal request")
        try:
            x = request.data.get('x', 0.0)
            y = request.data.get('y', 0.0)
            logger.info(f"Received coordinates: x={x}, y={y}")
            from ros2_core.ros2_executor import publish_goal
            publish_goal(x, y)
            return Response({'status': 'Goal sent successfully'}, status=status.HTTP_200_OK)
        except Exception as e:
            logger.error(f"Error occurred: {e}")
            return Response({'error': str(e)}, status=status.HTTP_400_BAD_REQUEST)
