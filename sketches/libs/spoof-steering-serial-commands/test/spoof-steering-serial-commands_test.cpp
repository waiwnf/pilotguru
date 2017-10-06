// Unit tests for KiaControlCommandProcessor parser.

#include "gtest/gtest.h"

#include <spoof-steering-serial-commands.h>

namespace pilotguru {
namespace kia {
namespace {

bool CommandsEqual(const KiaControlCommand &a, const KiaControlCommand &b) {
  return (a.type == b.type) && (a.value == b.value);
}

class KiaControlCommandProcessorTest : public ::testing::Test {
protected:
  KiaControlCommandProcessor::CommandStatus
  ConsumeString(const char *in_string) {
    for (size_t i = 0; i < strlen(in_string); ++i) {
      command_processor_.ConsumeChar(in_string[i]);
    }
    return command_processor_.GetCommandStatus();
  }

  KiaControlCommandProcessor command_processor_;
  KiaControlCommand command_;
};

TEST_F(KiaControlCommandProcessorTest, SteerPositive) {
  EXPECT_EQ(ConsumeString("s128\r"), KiaControlCommandProcessor::READY_OK);
  EXPECT_TRUE(command_processor_.GetCurrentCommand(&command_));
  EXPECT_TRUE(CommandsEqual(command_, {KiaControlCommand::STEER, 128}));
}

TEST_F(KiaControlCommandProcessorTest, SteerNegative) {
  EXPECT_EQ(ConsumeString("s-256\r"), KiaControlCommandProcessor::READY_OK);
  EXPECT_TRUE(command_processor_.GetCurrentCommand(&command_));
  EXPECT_TRUE(CommandsEqual(command_, {KiaControlCommand::STEER, -256}));
}

TEST_F(KiaControlCommandProcessorTest, SteerNoMagnitude) {
  EXPECT_EQ(ConsumeString("s\r"), KiaControlCommandProcessor::PARSE_FAIL);
  EXPECT_FALSE(command_processor_.GetCurrentCommand(&command_));
}

TEST_F(KiaControlCommandProcessorTest, SteerIntOverflow) {
  EXPECT_EQ(ConsumeString("s999999\r"),
            KiaControlCommandProcessor::PARSE_FAIL);
  EXPECT_FALSE(command_processor_.GetCurrentCommand(&command_));
}

TEST_F(KiaControlCommandProcessorTest, SteerNonDecimalIntSuffix) {
  EXPECT_EQ(ConsumeString("s0xFF\r"), KiaControlCommandProcessor::PARSE_FAIL);
  EXPECT_FALSE(command_processor_.GetCurrentCommand(&command_));
}

TEST_F(KiaControlCommandProcessorTest, SteerBufferOverflow) {
  EXPECT_EQ(ConsumeString("s000000000000000000000000001\r"),
            KiaControlCommandProcessor::COMMAND_OVERFLOW);
  EXPECT_FALSE(command_processor_.GetCurrentCommand(&command_));
}

TEST_F(KiaControlCommandProcessorTest, SteerIncomplete) {
  // No \r in the end -> command not finished yet.
  EXPECT_EQ(ConsumeString("s128"), KiaControlCommandProcessor::INCOMPLETE);
  EXPECT_FALSE(command_processor_.GetCurrentCommand(&command_));
}

TEST_F(KiaControlCommandProcessorTest, EchoOn) {
  EXPECT_EQ(ConsumeString("e001\r"), KiaControlCommandProcessor::READY_OK);
  EXPECT_TRUE(command_processor_.GetCurrentCommand(&command_));
  EXPECT_TRUE(CommandsEqual(command_, {KiaControlCommand::ECHO_COMMAND, 1}));
}

TEST_F(KiaControlCommandProcessorTest, EchoOff) {
  EXPECT_EQ(ConsumeString("e0\r"), KiaControlCommandProcessor::READY_OK);
  EXPECT_TRUE(command_processor_.GetCurrentCommand(&command_));
  EXPECT_TRUE(CommandsEqual(command_, {KiaControlCommand::ECHO_COMMAND, 0}));
}

TEST_F(KiaControlCommandProcessorTest, EchoOtherValue) {
  EXPECT_EQ(ConsumeString("e05\r"), KiaControlCommandProcessor::PARSE_FAIL);
  EXPECT_FALSE(command_processor_.GetCurrentCommand(&command_));
}

TEST_F(KiaControlCommandProcessorTest, EchoMoValue) {
  EXPECT_EQ(ConsumeString("e\r"), KiaControlCommandProcessor::PARSE_FAIL);
  EXPECT_FALSE(command_processor_.GetCurrentCommand(&command_));
}

TEST_F(KiaControlCommandProcessorTest, UnknownCommand) {
  EXPECT_EQ(ConsumeString("z128\r"), KiaControlCommandProcessor::PARSE_FAIL);
  EXPECT_FALSE(command_processor_.GetCurrentCommand(&command_));
}

TEST_F(KiaControlCommandProcessorTest, UnknownCommandNoValue) {
  EXPECT_EQ(ConsumeString("z\r"), KiaControlCommandProcessor::PARSE_FAIL);
  EXPECT_FALSE(command_processor_.GetCurrentCommand(&command_));
}

}
}
}
