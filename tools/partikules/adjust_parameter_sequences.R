## Load sequence

filename <- "static-cells-input-sequence.csv"
sequence <- read.table (filename, stringsAsFactors = FALSE)

head (sequence, 50)
c (min (sequence$V1), max (sequence$V1))

## Synchronise sequence

sync <- function (from, to, delay) {
    sequence [sequence$V1 >= from & sequence$V1 <= to, "V1"] <<- sequence [sequence$V1 >= from & sequence$V1 <= to, "V1"] + delay
}

scale <- function (from1, to1, from2, to2) {
    subseq <- sequence [sequence$V1 >= from1 & sequence$V1 <= to1, "V1"]
    sequence [sequence$V1 >= from1 & sequence$V1 <= to1, "V1"] <<- (subseq - from1) / (to1 - from1) * (to2 - from2) + from2
}

sync (177, 245, -1.5)
scale (1, 52.1, 1.5, 52.1)
scale (174.73, 220, 174.73, 245)

head (sequence, 50)
c (min (sequence$V1), max (sequence$V1))


## Shift value
shift <- function (type, fromTime, toTime, offset) {
    sequence [sequence$V1 >= fromTime & sequence$V1 <= toTime & sequence$V2 == type, "V3"] <<- sequence [sequence$V1 >= fromTime & sequence$V1 <= toTime & sequence$V2 == type, "V3"] + offset
}

shift ("bodyX", from, to, -0.3)
shift ("bodyX", 82.5, 83.8, 0.3)


## Remove events
remove <- function (type, fromTime, toTime) {
    sequence <<- sequence [sequence$V1 < fromTime | sequence$V1 > toTime | sequence$V2 != type, ]
}

remove ("bodyY", 17, 32)
remove ("pixelIntensity", 223, 250)
remove ("pixelIntensity", 222, 250)


## Add event

linear <- function (type, fromTime, toTime, fromValue, toValue, stepPerSec = 30) {
    stepNumber <- floor ((toTime - fromTime) * stepPerSec)
    times <- seq(stepNumber) / stepNumber * (toTime - fromTime) + fromTime
    values <- seq(stepNumber) / stepNumber * (toValue - fromValue) + fromValue
    types <- rep (type, stepNumber)

    df <- data.frame (times, types, values, stringsAsFactors=FALSE)
    names (df) <- c("V1","V2","V3")
    sequence <<- rbind (sequence, df)
}

linear ("pixelIntensity", 0.75, 3, 0, 1)
linear ("pixelIntensity", 223, 229, 1, 0)
linear ("bodyY", 28, 32, 0.4292, 0.2083)
linear ("bodyY", 17, 32, 0.37, 0.2083)

linear ("pixelIntensity", 3*60+42, 3*60+45, 1, 0)

## Reorder sequence

sequence <- sequence [order(sequence$V1), ]
head (sequence, -50)


## Cut precision of time and value

sequence$V1 <- round (sequence$V1, 5)
sequence$V3 <- round (sequence$V3, 5)


## Save sequence

head (sequence, 50)
write.table (sequence, filename, col.names=FALSE, row.names=FALSE, quote=FALSE)



## Merge two wequences

inputFile <- "static-cells-input-sequence.csv"
##outputFile <- "static-cells-output-sequence.csv"
outputFile <- "static-cells-input-sequence-144.csv"

input <- read.table (inputFile, stringsAsFactors = FALSE)
input$V1 <- input$V1 - 0
head (input, 30)

output <- read.table (outputFile, stringsAsFactors = FALSE)
output$V1 <- output$V1 + 0
head (output, 20)

input <- rbind (input, output)
input <- input [order(input$V1), ]
head (input, 100)
input

write.table (input, "static-cells-input-sequence.csv", col.names=FALSE, row.names=FALSE, quote=FALSE)



library("plyr")
## Clean sequence
inputFile <- "static-cells-input-sequence.csv"
input <- read.table (inputFile, stringsAsFactors = FALSE)
input$V3 <- ifelse (abs(input$V3) < 0.0000001, 0, input$V3)
input$V3 <- as.character(input$V3)
head (input, 40)
nrow(input)

input$V4 <- 0
currentBodyWeight <- 0
for (i in 1:nrow(input)) {
    if (input[i,2] == "bodyWeight") { currentBodyWeight <- input[i,3] }
    input[i,4] <- currentBodyWeight
}

input$V5 <- 0
currentStep <- 1
currentBodyWeight <- 0
for (i in 1:nrow(input)) {
    if (input[i,4] != currentBodyWeight || currentBodyWeight != 0) {
        currentStep <- currentStep + 1
        currentBodyWeight <- input[i,4]
    }
    input[i,5] <- currentStep
}

head (input, 200)

inputBodyX <- input [input$V2 == "bodyX", ]
inputBodyX <- ddply (inputBodyX, 5, function (x) x[nrow(x),])
inputBodyX <- ddply (inputBodyX, 1, function (x) x[nrow(x),])
inputBodyX$V6 <- append(TRUE, inputBodyX[2:nrow(inputBodyX),3] != inputBodyX[1:(nrow(inputBodyX)-1),3])
inputBodyX <- inputBodyX[inputBodyX$V6, c(1,2,3)]
head(inputBodyX,60)

inputBodyY <- input [input$V2 == "bodyY", ]
inputBodyY <- ddply (inputBodyY, 5, function (x) x[nrow(x),])
inputBodyY <- ddply (inputBodyY, 1, function (x) x[nrow(x),])
inputBodyY$V6 <- append(TRUE, inputBodyY[2:nrow(inputBodyY),3] != inputBodyY[1:(nrow(inputBodyY)-1),3])
inputBodyY <- inputBodyY[inputBodyY$V6, c(1,2,3)]
head(inputBodyY,60)

input <- input[, c(1,2,3)]
input <- input[! input$V2 %in% c("bodyX","bodyY"), ]
input <- rbind (input, inputBodyX, inputBodyY)
input <- input [order(input$V1), ]
head (input, 70)
nrow(input)

write.table (input, "static-cells-input-sequence.csv", col.names=FALSE, row.names=FALSE, quote=FALSE)

    
## Other
filename <- "static-cells-parameter-sequence-1.csv"


## load sequence
sequence <- read.table (filename, stringsAsFactors = FALSE)
head (sequence, 50)

## remove bodyX and bodyY
##sequence <- sequence [! sequence$V2 %in% c("bodyX","bodyY"), ]

## translate time
time <- 1.13513
sequence$V1 <- sequence$V1 - time
sequence$V1 <- pmax (0, sequence$V1)
##sequence <- sequence [sequence$V1 > 0, ]

head (sequence, 50)
sequence

## save new sequence
write.table (sequence, filename, col.names=FALSE, row.names=FALSE, quote=FALSE)





## Smooth sequence

filename <- "static-cells-input-sequence.csv"
sequence <- read.table (filename, stringsAsFactors = FALSE)
head (sequence)

smooth <- function (type, minDuration, maxDuration) {
    seqType <- sequence [sequence$V2 == type, ]
    seqType$V4 <- seqType$V1 - append (0, seqType[1:(nrow(seqType)-1),"V1"])

    hist (seqType[seqType$V4 < maxDuration, "V4"], breaks=100)

    for (r in 1:nrow(seqType)) {
        X <- seqType[r,]
        if (X$V4 < maxDuration && X$V4 > minDuration) {
            fromValue <- seqType[r-1,"V3"]
            toValue <- seqType[r,"V3"]
            fromTime <- seqType[r-1,"V1"]
            toTime <- seqType[r,"V1"]

            steps <- ceiling ((toTime - fromTime) / minDuration)
            times <- (1:(steps-1)) * (toTime - fromTime) / steps + fromTime
            types <- rep(type,steps-1)
            values <- (1:(steps-1)) * (toValue - fromValue) / steps + fromValue

            df <- data.frame (times, types, values, stringsAsFactors=FALSE)
            names (df) <- c("V1","V2","V3")
            sequence <<- rbind (sequence, df)
        }
    }

    sequence <<- sequence [order(sequence$V1), ]
}


seqType <- sequence [sequence$V2 == "bodyX", ]
seqType$V4 <- seqType$V1 - append (0, seqType[1:(nrow(seqType)-1),"V1"])
hist (seqType[seqType$V4 < 0.8, "V4"], breaks=100)

smooth ("bodyX", 1/30, 0.8)
smooth ("bodyY", 1/30, 0.8)

write.table (sequence, filename, col.names=FALSE, row.names=FALSE, quote=FALSE)
