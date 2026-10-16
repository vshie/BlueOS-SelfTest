FROM python:3.11-slim-bookworm

# iputils-ping: ICMP sweep for the network device scan
# curl: fetch frontend vendor assets at build time
RUN apt-get update -o Acquire::Retries=3 \
    && apt-get install -y --no-install-recommends --fix-missing \
        curl iputils-ping \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

RUN pip install --no-cache-dir flask==3.0.3 && \
    pip install --no-cache-dir requests==2.32.3 && \
    pip install --no-cache-dir waitress==3.0.0

# Per-run test logs live here; bound to /usr/blueos/extensions/selftest on the
# host so they show up in the BlueOS File Browser under extensions/selftest.
RUN mkdir -p /app/logs && chmod 777 /app/logs

COPY scripts/fetch_vendor.sh /tmp/fetch_vendor.sh
COPY app/ .

# Vendor Vue/Vuetify/MDI/Roboto at build time so the UI works with no internet
# (vehicles on a tether usually have none).
RUN sh /tmp/fetch_vendor.sh /app/static/vendor && rm /tmp/fetch_vendor.sh

ENV PYTHONUNBUFFERED=1
EXPOSE 9450

LABEL version="1.3.0"
LABEL type="tool"
LABEL requirements="core >= 1.1"

ARG IMAGE_NAME
LABEL permissions='\
{\
  "ExposedPorts": {\
    "9450/tcp": {}\
  },\
  "HostConfig": {\
    "CpuPeriod": 100000,\
    "CpuQuota": 100000,\
    "Binds": [\
      "/usr/blueos/extensions/selftest:/app/logs"\
    ],\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "NetworkMode": "host"\
  }\
}'

ARG AUTHOR
ARG AUTHOR_EMAIL
LABEL authors='[\
    {\
        "name": "Tony White",\
        "email": "tonywhite@bluerobotics.com"\
    }\
]'

ARG MAINTAINER
ARG MAINTAINER_EMAIL
LABEL company='{\
    "about": "Guided hardware self-test for BlueROV2 and BlueBoat",\
    "name": "Blue Robotics",\
    "email": "support@bluerobotics.com"\
}'

LABEL tags='[\
    "self-test",\
    "diagnostics",\
    "production",\
    "bluerov2",\
    "blueboat",\
    "calibration",\
    "sonar"\
]'

ARG REPO
ARG OWNER
LABEL readme='https://raw.githubusercontent.com/vshie/BlueOS-SelfTest/{tag}/README.md'
LABEL links='{\
    "source": "https://github.com/vshie/BlueOS-SelfTest",\
    "website": "https://bluerobotics.com",\
    "support": "mailto:support@bluerobotics.com"\
}'

CMD ["python", "main.py"]
