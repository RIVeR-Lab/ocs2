/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <exception>

#include <ocs2_core/integration/SystemEventHandler.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<bool, size_t> SystemEventHandler::checkEvent(OdeBase& system, scalar_t time, const vector_t& state) 
{
  //std::cout << "[SystemEventHandler::checkEvent] START" << std::endl;

  //std::cout << "[SystemEventHandler::checkEvent] state size: " << state.size() << std::endl;
  //std::cout << state << std::endl;

  /*
  if (state.size() != 9)
  {
    std::cout << "[SystemEventHandler::checkEvent] WATCHOUT: " << state.size() << std::endl;
    //killIntegration_ = true;
    //return {true, 1};
  }
  */

  //std::cout << "[SystemEventHandler::checkEvent] END" << std::endl;

  return {false, 0};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SystemEventHandler::handleEvent(OdeBase& system, scalar_t time, const vector_t& state) 
{
  //std::cout << "[SystemEventHandler::handleEvent] START" << std::endl;

  if (killIntegration_) 
  {
    std::cout << "[SystemEventHandler::handleEvent] killIntegration_ TRUE" << std::endl;
    //std::cout << "[SystemEventHandler::handleEvent] DEBUG INF" << std::endl;
    //while(1);

    //throw std::runtime_error("[SystemEventHandler::handleEvent] Integration terminated due to an external signal triggered by a program.");
  }

  // derived class events
  size_t eventID;
  bool event;
  std::tie(event, eventID) = this->checkEvent(system, time, state);
  if (event) 
  {
    //std::cout << "[SystemEventHandler::handleEvent] WTF" << std::endl;
    //throw eventID;
  }

  //std::cout << "[SystemEventHandler::handleEvent] END" << std::endl;
}

}  // namespace ocs2
