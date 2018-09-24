rm (list = ls ())

fitnessFile <- "fitness.csv"
selectedFitness <- c(4,5)
parentNb <- 2
mutationProb <- 0.1

oldfilename <- "config/done/random-config"
oldfirst <- 200
oldlast <- 249

newfilename <- "config/random-config"
newfirst <- 250
newlast <- 299


## Program
fitness <- read.csv (fitnessFile)
names (fitness) <- c ("num", "fitness")
fitness <- fitness [oldfirst:oldlast, ]

fitness <- fitness [fitness$fitness %in% selectedFitness, ]
cat ("->", nrow (fitness), "selected samples\n")


get_param <- function (param) {
    if (runif (1, 0, 1) < mutationProb) {
        value <- switch (param,
                         particleNumber = as.integer (100000),
                         particleSpeed = runif (1, 0, 0.01),
                         particleDamping = runif (1, 0, 0.1),
                         gravitationFactor = runif (1, -5, 5),
                         gravitationAngle = runif (1, 0, 360),
                         bodyRadius = 0,
                         bodyAttractFactor = 1,
                         bodyRepelFactor = 1,
                         withFixedBody = 1,
                         fixedBodyX = runif (1, 0, 1),
                         fixedBodyY = runif (1, 0, 1),
                         fixedBodyWeight = runif (1, 0, 10)
                         )
    } else {
        oldnum <- sample (configs, 1)
        oldnumStr <- paste0 (paste (rep ("0", 6 - nchar (as.character (oldnum))), collapse=""), oldnum)
        oldfile <- paste (oldfilename, oldnumStr, sep = "-")
        values <- read.csv (oldfile, sep=" ", header=FALSE, colClasses=c("character","character"))
        values$V2 <- as.character (values$V2)
        value <- values [values$V1 == param, "V2"]
    }

    cat (param, value, "\n", file = newfile, append=TRUE)
}

for (newnum in newfirst:newlast) {
    newnumStr <- paste0 (paste (rep ("0", 6 - nchar (as.character (newnum))), collapse=""), newnum)
    newfile <- paste (newfilename, newnumStr, sep = "-")
    configs <- fitness [sample (nrow (fitness), parentNb), "num"]

    cat ("", file = newfile)
    
    get_param ("particleNumber")
    get_param ("particleSpeed")
    get_param ("particleDamping")
    get_param ("gravitationFactor")
    get_param ("gravitationAngle")
    get_param ("bodyRadius")
    get_param ("bodyAttractFactor")
    get_param ("bodyRepelFactor")
    get_param ("withFixedBody")
    get_param ("fixedBodyX")
    get_param ("fixedBodyY")
    get_param ("fixedBodyWeight")
}

