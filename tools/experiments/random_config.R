filename <- "config/random-config"
firstnum <- 20
lastnum <- 49

for (num in firstnum:lastnum) {
    particleNumber <- as.integer (100000)
    particleSpeed <- runif (1, 0, 0.01)
    particleDamping <- runif (1, 0, 0.1)
    gravitationFactor <- runif (1, -5, 5)
    gravitationAngle <- runif (1, 0, 360)
    bodyRadius <- 0
    bodyAttractFactor <- 1
    bodyRepelFactor <- 1
    withFixedBody <- 1
    fixedBodyX <- runif (1, 0, 1)
    fixedBodyY <- runif (1, 0, 1)
    fixedBodyWeight <- runif (1, 0, 10)

    numStr <- paste0 (paste (rep ("0", 6 - nchar (as.character (num))), collapse=""), num)
    file <- paste (filename, numStr, sep = "-")
    cat ("particleNumber", particleNumber, "\n", file = file)
    cat ("particleSpeed", particleSpeed, "\n", file = file, append = TRUE)
    cat ("particleDamping", particleDamping, "\n", file = file, append = TRUE)
    cat ("gravitationFactor", gravitationFactor, "\n", file = file, append = TRUE)
    cat ("gravitationAngle", gravitationAngle, "\n", file = file, append = TRUE)
    cat ("bodyRadius", bodyRadius, "\n", file = file, append = TRUE)
    cat ("bodyAttractFactor", bodyAttractFactor, "\n", file = file, append = TRUE)
    cat ("bodyRepelFactor", bodyRepelFactor, "\n", file = file, append = TRUE)
    cat ("withFixedBody", withFixedBody, "\n", file = file, append = TRUE)
    cat ("fixedBodyX", fixedBodyX, "\n", file = file, append = TRUE)
    cat ("fixedBodyY", fixedBodyY, "\n", file = file, append = TRUE)
    cat ("fixedBodyWeight", fixedBodyWeight, "\n", file = file, append = TRUE)
}
