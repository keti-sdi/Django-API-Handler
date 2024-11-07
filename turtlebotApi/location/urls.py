from django.urls import path
from .views import StartPosePublishingView, StopPosePublishingView, TurtlebotPoseView

urlpatterns = [
    path('start-publishing/', StartPosePublishingView.as_view(), name='start-publishing'),
    path('stop-publishing/', StopPosePublishingView.as_view(), name='stop-publishing'),
    path('current-pose/', TurtlebotPoseView.as_view(), name='current-pose'),
]
