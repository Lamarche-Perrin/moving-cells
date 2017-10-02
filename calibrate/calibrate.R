
data <- read.table("calibrate.input", sep=";", header=TRUE)

axe <- "y"
type <- "Max"
var <- "V1"
write("","calibrate.output")

for (axe in c("x","y")) { #for (axe in c("x","y","z")) {
    for (type in c("Min","Moy","Max")) {
        for (var in c("V1","V2")) {
            
            d <- data[data$AXE == axe & data$TYPE == type, c("X","Y","Z",var)]
            pairs(d)
            colnames(d)[4] <- "V"
                                        #pairs(d)

            model <- lm(V ~ Z + I(Z^2), data = d)
            summary(model)
            model
            
            if (axe == "x" && var == "V1") { str <- "Left" }
            if (axe == "x" && var == "V2") { str <- "Right" }
            if (axe == "y" && var == "V1") { str <- "Top" }
            if (axe == "y" && var == "V2") { str <- "Bottom" }
            if (axe == "z" && var == "V1") { str <- "Front" }
            if (axe == "z" && var == "V2") { str <- "Back" }
            newvar <- paste(axe,str,type, sep="")

            ## linear Z
            write(paste("float",newvar,"=",model[[1]][1],"+ body.zMoy *",model[[1]][2],";"), file = "calibrate.output", append=TRUE)

            # quad Z
            write(paste("float",newvar,"=",model[[1]][1],"+ body.zMoy *",model[[1]][2],"+ pow(body.zMoy,2) *",model[[1]][3],";"), file = "calibrate.output", append=TRUE)

            # cube Z
            write(paste("float",newvar,"=",model[[1]][1],"+ body.zMoy *",model[[1]][2],"+ pow(body.zMoy,2) *",model[[1]][3],"+ pow(body.zMoy,3) *",model[[1]][4],";"), file = "calibrate.output", append=TRUE)
            
            write(paste("float",newvar,"=",model[[1]][1],"+ body.xMoy *",model[[1]][2],"+ body.yMoy *",model[[1]][3],"+ body.zMoy *",model[[1]][4],";"), file = "calibrate.output", append=TRUE)
            
        }
    }
}



for (axe in c("x","y")) { #for (axe in c("x","y","z")) {
    for (type in c("Min","Moy","Max")) {
        for (var in c("V1","V2")) {
            
            d <- data[data$AXE == axe & data$TYPE == type, c("X","Y","Z",var)]
            colnames(d)[4] <- "V"
                                        #pairs(d)

            model <- lm(V ~ Z, data = d)

            if (axe == "x" && var == "V1") { str <- "Left" }
            if (axe == "x" && var == "V2") { str <- "Right" }
            if (axe == "y" && var == "V1") { str <- "Top" }
            if (axe == "y" && var == "V2") { str <- "Bottom" }
            if (axe == "z" && var == "V1") { str <- "Front" }
            if (axe == "z" && var == "V2") { str <- "Back" }
            newvar <- paste(axe,str,type, sep="")
            
            write(paste("float",newvar,"=",model[[1]][1],"+ body.zMoy *",model[[1]][2],";"), file = "calibrate.output", append=TRUE)
            
        }
    }
}
