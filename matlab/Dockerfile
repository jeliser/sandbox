FROM mathworks/matlab:r2021a

WORKDIR /home/matlab

COPY .local/MATLAB/R2021a /opt/matlab/R2021a

RUN sudo apt-get update
RUN sudo apt-get install -y firefox
RUN sudo apt-get install -y gedit
