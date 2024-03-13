FROM ros:humble

SHELL ["/bin/bash", "-c"]

# install basic tools

RUN apt-get update && apt-get -y install \
    vim wget curl \
    libopencv-dev ros-humble-cv-bridge\
    python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY ./requirements.txt ./
RUN pip install -r ./requirements.txt && \
    rm -f ./requirements.txt

# install oh my zsh & change theme to af-magic
RUN apt-get update && apt-get -y install zsh 
RUN wget https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh -O zsh-install.sh && \
    chmod +x ./zsh-install.sh && ./zsh-install.sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    sed -i 's/plugins=(git)/plugins=(git zsh-syntax-highlighting zsh-autosuggestions)/g' ~/.zshrc  && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    rm ./zsh-install.sh 

RUN chsh root -s /bin/zsh