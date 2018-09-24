rm (list = ls ())

fitnessFile <- "fitness.csv"
selectedFitness <- c(4,5)

filename <- "config/done/random-config"
first <- 0
last <- 249


## Program
fitness <- read.csv (fitnessFile)
names (fitness) <- c ("num", "fitness")
fitness <- fitness [first:last, ]

fitness <- fitness [fitness$fitness %in% selectedFitness, ]
cat ("->", nrow (fitness), "selected samples\n")


param <- list ()
for (num in fitness$num) {
    numStr <- paste0 (paste (rep ("0", 6 - nchar (as.character (num))), collapse=""), num)
    file <- paste (filename, numStr, sep = "-")
    values <- read.csv (file, sep=" ", header=FALSE, stringsAsFactors=FALSE)
    for (i in 1:nrow(values)) {
        name <- values [i, "V1"]
        value <- values [i, "V2"]
        if (is.null (param[[name]])) { param[[name]] <- c() }
        param[[name]] <- append (param[[name]], value)
    }
}

names (param)
indices <- c(2,3,4,5,10,11,12)

par (mfrow=c(2,4))
for (i in indices) {
    hist (param[[i]], xlab=names(param)[i], main=paste("Histogram of" , names(param)[i]))
}
    

param <- as.data.frame (param)
param <- param[, indices]
pairs (sapply (param, jitter, amount=1))
param$particleSpeed
?pairs
