FROM quay.io/coreos/etcd:v3.5.10
ENV ALLOW_NONE_AUTHENTICATION=yes
CMD ["etcd", \
"--name", "emulator_etcd", \
"--advertise-client-urls", "http://0.0.0.0:2379", \
"--listen-client-urls", "http://0.0.0.0:2379", \
"--listen-peer-urls", "http://0.0.0.0:2380", \
"--initial-advertise-peer-urls", "http://0.0.0.0:2380", \
"--initial-cluster", "emulator_etcd=http://0.0.0.0:2380", \
"--data-dir", "/etcd-data"]
EXPOSE 2379
