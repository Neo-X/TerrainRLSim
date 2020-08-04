FROM nvidia/cuda:8.0-cudnn6-devel-ubuntu16.04

SHELL ["/bin/bash", "-c"]

RUN apt-get update -y
# RUN apt-get install -y python3-dev python3-pip
RUN apt-get update --fix-missing
RUN apt-get install -y wget bzip2 ca-certificates git vim
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential \
        premake4 \
        git \
        curl \
        vim \
	swig \
	software-properties-common \
	xpra \
	net-tools \
	wget \
	cmake \
	xserver-xorg-dev \
    	zlib1g-dev \
        libav-tools \
	    libgl1-mesa-dev \
	    libgl1-mesa-glx \
	    libglew-dev \
	    libosmesa6-dev \
	    libxrender-dev \
	    libsm6 libxext6 \
        unzip \
        patchelf \
        ffmpeg \
        graphviz \
        libxrandr2 \
        libxinerama1 \
        libxcursor1 \
        python3-dev python3-pip graphviz \
        freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev libgl1-mesa-glx libglu1-mesa libglu1-mesa-dev libglew1.6-dev mesa-utils


# Not sure why this is needed
ENV LANG C.UTF-8

# Not sure what this is fixing
# COPY ./files/Xdummy /usr/local/bin/Xdummy
# RUN chmod +x /usr/local/bin/Xdummy

# Workaround for https://bugs.launchpad.net/ubuntu/+source/nvidia-graphics-drivers-375/+bug/1674677
# COPY ./files/10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Not sure why this is needed
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

ENV PATH /opt/conda/bin:$PATH
RUN wget --quiet https://repo.anaconda.com/archive/Anaconda2-2019.10-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    /bin/bash /tmp/miniconda.sh -b -p /opt/conda && \
    rm /tmp/miniconda.sh && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> /etc/bash.bashrc

RUN conda update -y --name base conda && conda clean --all -y

RUN conda create --name rlframe python=3.6.9 pip
RUN echo "source activate rlframe" >> ~/.bashrc

ENV PATH /opt/conda/envs/rlframe/bin:$PATH

# RUN python -c 'import mujoco_py'

RUN mkdir /root/playground
WORKDIR /root/playground

COPY id_rsa_remote_client /root/.ssh/id_rsa
RUN chmod 600 ~/.ssh/id_rsa
# make sure your domain is accepted
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

# RUN ls
# RUN git clone https://github.com/rll/rllab.git /root/playground/rllab
# RUN git clone git@github.com:FracturedPlane/RLSimulationEnvironments.git /root/playground/RLSimulationEnvironments
# RUN pip install -r /root/playground/RLSimulationEnvironments/requirements.txt
# ENV RLSIMENV_PATH /root/playground/RLSimulationEnvironments

# RUN ls
# RUN git clone git@github.com:Neo-X/RL-Framework.git /root/playground/RL-Framework
# RUN pip install -r /root/playground/RL-Framework/requirements.txt
# RUN popd

### Add OpenGL libraries to LD path.
# COPY libOpenGL.so /usr/lib/nvidia/libOpenGL.so
ENV LD_LIBRARY_PATH /usr/lib/nvidia:${LD_LIBRARY_PATH}
ENV LD_LIBRARY_PATH /usr/lib:${LD_LIBRARY_PATH}

WORKDIR /root/playground
RUN git clone git@github.com:Neo-X/TerrainRLSim.git
ENV TERRAINRL_PATH /root/playground/TerrainRLSim
WORKDIR /root/playground/TerrainRLSim
RUN wget https://github.com/UBCMOCCA/TerrainRLSim/releases/download/0.8/TerrainRLSim_external_June_21_2019.tar.xz
RUN tar -xvf TerrainRLSim_external_June_21_2019.tar.xz
RUN chmod +x ./deb_deps.sh
RUN ./deb_deps.sh
RUN cd external/caffe && make clean && make -j 8
RUN cp -r external/caffe/build/lib . && cp external/caffe/build/lib/libcaffe.* lib/ && cp external/Bullet/bin/*.so lib/ && cp external/jsoncpp/build/debug/src/lib_json/*.so* lib/
WORKDIR /root/playground/TerrainRLSim/simAdapter
RUN chmod +x ./gen_swig.sh
RUN ./gen_swig.sh
WORKDIR /root/playground/TerrainRLSim/
RUN ls -la
# RUN chmod +x ./premake4_linux && ./premake4_linux --file=premake4_openglES.lua gmake
RUN chmod +x ./premake4_linux
RUN ./premake4_linux --file=premake4_openglES.lua gmake
### RUN cd gmake
WORKDIR /root/playground/TerrainRLSim/gmake
RUN make config=release64 -j 3
WORKDIR /root/playground/TerrainRLSim
RUN pip install -v -e $TERRAINRL_PATH
RUN pip install -r requirements.txt
WORKDIR /root/playground
RUN echo $LD_LIBRARY_PATH

RUN ls

##########################################################
### MuJoCo
##########################################################
# Note: ~ is an alias for /root
RUN mkdir -p /root/.mujoco \
    && wget https://www.roboti.us/download/mujoco200_linux.zip -O mujoco.zip \
    && unzip mujoco.zip -d /root/.mujoco \
    && rm mujoco.zip
RUN mkdir -p /root/.mujoco \
    && wget https://www.roboti.us/download/mjpro150_linux.zip -O mujoco.zip \
    && unzip mujoco.zip -d /root/.mujoco \
    && rm mujoco.zip
COPY ./files/mjkey.txt /root/.mujoco/mjkey.txt
RUN ln -s /root/.mujoco/mujoco200_linux /root/.mujoco/mujoco200
ENV LD_LIBRARY_PATH /root/.mujoco/mjpro150/bin:${LD_LIBRARY_PATH}
ENV LD_LIBRARY_PATH /root/.mujoco/mujoco200/bin:${LD_LIBRARY_PATH}
ENV LD_LIBRARY_PATH /root/.mujoco/mujoco200_linux/bin:${LD_LIBRARY_PATH}



##########################################################
### Python
##########################################################
ENV PATH /opt/conda/bin:$PATH
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    /bin/bash /tmp/miniconda.sh -b -p /opt/conda && \
    rm /tmp/miniconda.sh && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> /etc/bash.bashrc

# RUN conda update -y --name base conda && conda clean --all -y

# RUN conda create --name railrl python=3.6.5 pip
# RUN echo "source activate railrl" >> ~/.bashrc
# Use the railrl pip
ENV OLDPATH $PATH
ENV PATH /opt/conda/envs/rlframe/bin:$PATH

RUN pip install imagehash>=3.4
RUN pip install ipdb
RUN pip install Pillow>=4.0.0
RUN pip install pycparser>=2.17.0
RUN pip install pytest>=3.0.5
RUN pip install pytest-instafail==0.3.0
RUN pip install scipy>=0.18.0
RUN pip install glfw>=1.4.0
RUN pip install imageio>=2.1.2
RUN pip install opencv-python==3.4.0.12
RUN pip install sk-video==1.1.10
RUN pip install numpy-stl==2.7.0
RUN pip install pyquaternion==0.9.2
RUN pip install moviepy==0.2.3.5
RUN pip install scikit-image
RUN pip install gitpython==2.1.7
RUN pip install gtimer==1.0.0b5
RUN pip install joblib==0.9.4
RUN pip install dominate==2.3.1
RUN pip install path.py==10.3.1
RUN pip install cached-property==1.3.1
RUN pip install cloudpickle==1.3.0
RUN pip install matplotlib==2.2.2
RUN pip install pygame==1.9.6
RUN pip install awscli==1.11.179
RUN pip install boto3==1.4.8
RUN pip install python-dateutil==2.6.1
RUN pip install torch==1.1.0
RUN pip install torchvision

########
### Mujoco-py and gym
########
RUN curl -o /usr/local/bin/patchelf https://s3-us-west-2.amazonaws.com/openai-sci-artifacts/manual-builds/patchelf_0.9_amd64.elf \
    && chmod +x /usr/local/bin/patchelf
RUN pip install gym[all]==0.17.1
