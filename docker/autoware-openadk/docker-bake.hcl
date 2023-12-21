group "default" {
  targets = ["base", "devel", "prebuilt", "monorun"]
}

// For docker/metadata-action
target "docker-metadata-action-base" {}
target "docker-metadata-action-devel" {}
target "docker-metadata-action-prebuilt" {}
target "docker-metadata-action-monorun" {}

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

target "monorun" {
  inherits = ["docker-metadata-action-monorun"]
  dockerfile = "docker/autoware-openadk/Dockerfile"
<<<<<<< HEAD
  target = "monolithic"
}
=======
  target = "monorun"
}
>>>>>>> 5f13c741 (change monolithic to monorun, update readme and run.sh)
