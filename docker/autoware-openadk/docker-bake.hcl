group "default" {
  targets = ["base", "devel", "prebuilt", "monolithic"]
}

// For docker/metadata-action
target "docker-metadata-action-base" {}
target "docker-metadata-action-devel" {}
target "docker-metadata-action-prebuilt" {}
target "docker-metadata-action-monolithic" {}

target "base" {
  inherits = ["docker-metadata-action-base"]
  dockerfile = "docker/autoware-openadk/Dockerfile"
  target = "base"
}

target "devel" {
  inherits = ["docker-metadata-action-devel"]
  dockerfile = "docker/autoware-openadk/Dockerfile"
  target = "devel"
}

target "prebuilt" {
  inherits = ["docker-metadata-action-prebuilt"]
  dockerfile = "docker/autoware-openadk/Dockerfile"
  target = "prebuilt"
}

target "monolithic" {
  inherits = ["docker-metadata-action-monolithic"]
  dockerfile = "docker/autoware-openadk/Dockerfile"
  target = "monolithic"
}