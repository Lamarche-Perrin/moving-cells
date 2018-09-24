library ("png")

png <- readPNG ("distribution.oubli.png")

png <- png / sum(png)
png <- as.vector (png)

I <- sample (1:length(png), 1000000, replace=TRUE, prob=png)

Y <- (I-1) %% 1080
X <- ((I-1)-Y)/1080

write.table (data.frame (X, Y), "distribution.oubli.csv", col.names=FALSE, row.names=FALSE)
