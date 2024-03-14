// -*-c++-*--------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AVIARY__MESSAGE_PROCESSOR_HPP_
#define AVIARY__MESSAGE_PROCESSOR_HPP_

namespace aviary
{
template <typename T>
class MessageProcessor
{
public:
  virtual ~MessageProcessor() {}
  virtual void process(uint64_t t, const typename T::ConstSharedPtr & msg) = 0;
};
}  // namespace aviary

#endif  // AVIARY__MESSAGE_PROCESSOR_HPP_
