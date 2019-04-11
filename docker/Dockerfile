FROM ros:kinetic

# utils
RUN apt-get update && apt-get install --yes debconf-utils wget apache2

# preseed graphite remove database question
RUN echo "graphite-carbon graphite-carbon/postrm_remove_databases boolean false" | debconf-set-selections
# install graphite
RUN apt-get update && apt-get install --yes graphite-carbon graphite-web

# install grafana
RUN wget https://dl.grafana.com/oss/release/grafana_6.1.3_amd64.deb
RUN dpkg -i grafana_6.1.3_amd64.deb

