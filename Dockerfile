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
# ENV LD_LIBRARY_PATH /usr/lib/nvidia:${LD_LIBRARY_PATH}

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
RUN make config=release64 -j 8
WORKDIR /root/playground/TerrainRLSim
RUN pip install -v -e $TERRAINRL_PATH
RUN pip install -r requirements.txt
WORKDIR /root/playground
RUN echo $LD_LIBRARY_PATH

RUN ls
