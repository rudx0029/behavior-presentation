#pragma once
#include "types.hpp"
#include <cassert>
#include <functional>
#include <vector>

// C++ interface or abstract base class describing an element of behavior. The element has initialize
// and finalize methods acting as constructors/destructors allowing the object to be reused more than
// once, as in a loop. The tick method is invoked periodically and the element should perform its
// work there.
class BehaviorElement {
public:
  virtual ElementMeta initialize(Services) = 0;
  virtual Outcome tick(const SenseInfo &) = 0;
  virtual void finalize() = 0;
};

// A MotionElement is a class of BehaviorElements that actuate the robot. This is different
// than a SequenceElement since a SequenceElement is container operating on elements forming a logical
// expression.
//
// The MotionElement implements Continuously Recurring Template Pattern in order to take advantage of
// static-polymorphism. 
//
template <class Derived> class MotionElement : public BehaviorElement {
public:
  // Required Name Trait
  static constexpr char const *const NAME =
      nullptr; // Derived must specify name

  // Required Reaction Definition
  static constexpr ReactionDef KNEE_JERK_REACTION = ReactionDef_REQUIRED;
  static constexpr ReactionDef FLINCH_REACTION = ReactionDef_REQUIRED;

  // Overridable static methods (in place of virtual methods)
  static ElementMeta motion_element_initialize(Derived &me) {
    return ElementMeta{Derived::NAME};
  }
  static void motion_element_finalize(Derived &me) {}
  static void motion_element_data_initialize(Derived &me, const SenseInfo &in) {
  }

  ElementMeta initialize(Services svc) final {
    //////////////////////////////////////////
    // Compile Time Checks
    //////////////////////////////////////////

    // check that name has been specified
    static_assert(Derived::NAME != nullptr,
                  "The Derived BehaviorElement must specify the NAME trait");

    // compile-time check ensuring the derived specified all of the reaction
    // traits
    static_assert(
        Derived::FLINCH_REACTION != ReactionDef_REQUIRED,
        "FLINCH_REACTION trait must be specified in the derived class");
    static_assert(
        Derived::KNEE_JERK_REACTION != ReactionDef_REQUIRED,
        "KNEE_JERK_REACTION trait must be specified in the derived class");

    //////////////////////////////////////////
    // Run Time Implementation
    //////////////////////////////////////////

    // reset internal variables for continued reuse of the object
    m_first_tick = true;
    m_services = svc;

    // specify the muted reactions for the duration of the container and
    // sub-containers.
    m_services.reaction_svc.activate(get_reaction_defs());

    // call the statically overridable initialization
    return Derived::motion_element_initialize(derived());
  }

  Outcome tick(const SenseInfo &sense) final {
    if (m_first_tick) {
      m_first_tick = false;
      Derived::motion_element_data_initialize(derived(), sense);
    }

    return Derived::motion_element_tick(derived(), sense);
  }

  void finalize() final {
    // perform the derived finalization if it is supported
    Derived::motion_element_finalize(derived());

    // unmute reactions
    m_services.reaction_svc.release(get_reaction_defs());
  }

protected:
  Services::MessengerSvc &messenger() { return m_services.messenger; };

private:
  static constexpr uint32_t get_reaction_defs() {
    return Derived::KNEE_JERK_REACTION << 0 | Derived::FLINCH_REACTION << 1;
  }
  Derived &derived() { return static_cast<Derived &>(*this); }
  Services m_services{};
  bool m_first_tick{true}; // for data initialization during the first tick()
};

// A SequenceElement is a container of BehaviorElements that executes each
// container until complete. It will then iterate to the next container and
// execute it. If any container fails execution the SequenceElement will also
// end its execution and pass along the failure. The SequenceElement can be
// thought of as an 'AND' operation on a collection of elements.
class SequenceElement : public BehaviorElement {
public:
  using Elements = std::vector<std::reference_wrapper<BehaviorElement>>;

  SequenceElement(const Elements &el) : m_elements(el), m_iter(el.begin()) {}

  ElementMeta initialize(Services svc) override {
    m_svcs = svc;
    m_iter = m_elements.begin();
    m_new_element = true;
    return ElementMeta{"Sequence"};
  }

  Outcome tick(const SenseInfo &s) {
    Outcome o;
    o.value = Outcome::Return::Running;

    if (m_iter != m_elements.end()) {
      if (m_new_element) {
        m_meta = m_iter->get().initialize(m_svcs);
        m_svcs.messenger.notify(m_meta.name, "initialize");
        m_new_element = false;
      }

      m_svcs.messenger.notify(m_meta.name, "tick");
      auto cur_o = m_iter->get().tick(s);

      if (cur_o.value != Outcome::Return::Running) {
        m_iter->get().finalize();
        m_svcs.messenger.notify(m_meta.name, "finalize");

        // go to the next element
        m_new_element = true;
        ++m_iter;
        if (m_iter != m_elements.end()) {
          if (cur_o.value == Outcome::Return::Fail) {
            // on failure we will end the sequence
            // the cur output can be forwared
            o = cur_o;
          } else {
            // on success maintain that this sequence elment is still running
            o.value = Outcome::Return::Running;
            o.actuate = cur_o.actuate;
          }
        } else {
          // we're done with all elements, set the output from the last element
          o = cur_o;
        }

      } else {
        o = cur_o;
      }
    } else {
      // we arrive here when the sequence never had any elements
      o.value = Outcome::Return::Fail;
    }

    return o;
  }

  void finalize() override {}

private:
  Services m_svcs{};
  Elements m_elements{};
  Elements::const_iterator m_iter{};
  bool m_new_element = true;
  ElementMeta m_meta;
};
