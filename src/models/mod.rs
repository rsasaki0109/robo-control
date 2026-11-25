//! Robot plant models implementing [`crate::model::Dynamics`].

mod arm;
mod bicycle;
mod cartpole;
mod holonomic;
mod quadrotor;
mod unicycle;

pub use arm::PlanarArmDyn;
pub use bicycle::Bicycle;
pub use cartpole::CartPole;
pub use holonomic::Holonomic;
pub use quadrotor::Quadrotor2D;
pub use unicycle::Unicycle;
