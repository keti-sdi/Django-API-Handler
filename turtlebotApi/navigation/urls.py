# urls.py
from django.urls import path
from .views import GoalAPIView as DetailedGoalAPIView
from .views_normal import GoalAPIView as NormalGoalAPIView
from .views_shell import GoalAPIView as ShellGoalAPIView

urlpatterns = [
    path('nowposition-goal/', DetailedGoalAPIView.as_view(), name='detailed_goal'),
    path('normal-goal/', NormalGoalAPIView.as_view(), name='normal_goal'),
    path('turminal-goal/', ShellGoalAPIView.as_view(), name='shell_goal'),
]
