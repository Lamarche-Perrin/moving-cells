rm (list = ls())
library (dplyr)

duration <- 1
sampling <- 44000
    
sampleNb <- sampling * duration
time <- 1:sampleNb / sampling

get.wave <- function (ampl, freq) { return (ampl * cos (2*pi*freq*time)) }
add.wave <- function (wave, wave2) { return (wave + wave2) }
am.wave <- function (wave, ampl, freq) { return (wave * get.wave (ampl, freq)) }
fm.wave <- function (wave, ampl, freq, delta.freq = freq) { return (ampl * cos (2 * pi * freq * time+ 2 * pi * delta.freq * (cumsum (wave) / sampling))) }
plot.wave <- function (wave) { plot (time, wave, type="l", ylim = c(-max(abs(wave)),max(abs(wave)))) }

carrier <- get.wave (1, 50)
plot.wave (carrier)

wave <- get.wave (0.5, 2) %>% add.wave (get.wave (1, 0))
plot.wave (wave)

wave %>% plot.wave
wave %>% am.wave (1, 20) %>% plot.wave
wave %>% am.wave (1, 20) %>% fm.wave (1, 20, 40) %>% plot.wave

wave %>% fm.wave (1, 20, 80) %>% plot.wave
wave %>% fm.wave (1, 20, 80) %>% fm.wave (1, 20) %>% plot.wave
