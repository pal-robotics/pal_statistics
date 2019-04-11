FROM ros:kinetic

RUN apt-get update && apt-get install --yes debconf-utils

# preseed graphite remove database question
RUN echo "graphite-carbon graphite-carbon/postrm_remove_databases boolean false" | debconf-set-selections
# install graphite
RUN apt-get update && apt-get install --yes graphite-carbon graphite-web

