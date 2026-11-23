#pragma once

#include <BotOp/bot.h>

bool sense_HsvBlob(BotOp& bot, rai::Configuration& C,
                   const char* camName, const char* blobName,
                   const arr& hsvFilter, const arr& Pinv={}, int verbose=0);
