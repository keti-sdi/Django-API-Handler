FROM python:3.8.10 AS python-stage

WORKDIR /usr/src/app

COPY . .

RUN pip install --upgrade pip
RUN pip install -r requirements.txt

FROM osrf/ros:foxy-desktop

WORKDIR /usr/src/app

COPY --from=python-stage /usr/local/lib/python3.8/site-packages /usr/local/lib/python3.8/site-packages
COPY --from=python-stage /usr/src/app /usr/src/app

ENV PYTHONPATH="/usr/local/lib/python3.8/site-packages:$PYTHONPATH"
#ENV LD_LIBRARY_PATH="/opt/ros/foxy/lib:$LD_LIBRARY_PATH"

RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc \
    && echo "source ~/colcon_ws/install/local_setup.bash" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=30" >> /root/.bashrc \
    && echo "export ROS_MASTER_URI=http://10.0.5.53:11311" >> /root/.bashrc \
    && echo "export ROS_HOSTNAME=10.0.5.53" >> /root/.bashrc \
    && echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc

ENV ROS_DOMAIN_ID=29
#ENV ROS_MASTER_URI=http://10.0.5.53:11311
#ENV ROS_HOSTNAME=10.0.5.53
ENV TURTLEBOT3_MODEL=burger

CMD ["python3", "manage.py", "runserver", "0.0.0.0:8000"]

EXPOSE 8000

           
