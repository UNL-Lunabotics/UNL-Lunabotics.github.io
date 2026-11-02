FROM ruby:slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /site

ENV BUNDLE_PATH=/usr/local/bundle

# Install dependencies first to take advantage of Docker's build cache.
COPY Gemfile Gemfile.lock ./
RUN bundle install

# Include the site for standalone use; Compose will mount your live source.
COPY . .

EXPOSE 4000 35729

CMD ["bundle", "exec", "jekyll", "serve", "--host", "0.0.0.0", "--port", "4000"]