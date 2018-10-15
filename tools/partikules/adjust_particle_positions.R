inputFile <- "particle-positions-106.csv"

input <- read.table (inputFile, stringsAsFactors = FALSE)
nrow (input)
head (input)

## Randomize particles
output <- input [sample.int (nrow (input)), ]
nrow (output)
head (output)

## Kill velocity
output$V3 <- 0
output$V4 <- 0

## Write file
write.table (output, inputFile, col.names=FALSE, row.names=FALSE, quote=FALSE)




## Add particles

factor <- 4
inputFile <- "particle-positions-0.csv"

input <- read.table (inputFile, stringsAsFactors = FALSE)
nrow (input)
head (input)

new <- nrow (input) * (factor - 1)
S1 <- sample (1:nrow(input), new, replace=TRUE)
V1 <- input[S1,"V1"]
S2 <- sample (1:nrow(input), new, replace=TRUE)
V2 <- input[S2,"V2"]

df <- data.frame (V1, V2, stringsAsFactors=FALSE)
df$V3 <- 0
df$V4 <- 0
head (df)

input <- rbind (input, df)
write.table (input, inputFile, col.names=FALSE, row.names=FALSE, quote=FALSE)
