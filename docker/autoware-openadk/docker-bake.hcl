group "default" {
  targets = ["devel", "monorun"]
}

// For docker/metadata-action
target "docker-metadata-action-devel" {}
target "docker-metadata-action-monorun" {}

target "devel" {
  inherits = ["docker-metadata-action-devel"]
  dockerfile = "docker/autoware-openadk/Dockerfile"
  target = "devel"
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
