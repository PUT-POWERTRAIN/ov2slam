# Debian Bookworm z Node.js i Pythonem
FROM debian:bookworm

# Instalacja Node.js 22 (repozytorium NodeSource)
RUN apt-get update && \
    apt-get install -y curl gnupg && \
    curl -fsSL https://deb.nodesource.com/setup_22.x | bash - && \
    apt-get install -y nodejs && \
    apt-get install -y python3 python3-pip python3-venv git vim nano sudo bash && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ARG dla user ID (przekazywany z docker-compose lub build-arg)
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=wojtess

# Tworzenie użytkownika
RUN groupadd -g ${GROUP_ID} ${USERNAME} && \
    useradd -u ${USER_ID} -g ${GROUP_ID} -m -s /bin/bash ${USERNAME} && \
    echo '${USERNAME} ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Instalacja Claude Code globalnie przez npm
RUN npm install -g @anthropic-ai/claude-code

# Instalacja bibliotek do symulacji plynow (CFD)
RUN pip3 install --break-system-packages --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    fluidsim \
    jax \
    jax-cfd \
    taichi

# Przelaczenie na uzytkownika
USER ${USERNAME}

# Domyślna komenda - bash interaktywny
CMD ["/bin/bash"]
