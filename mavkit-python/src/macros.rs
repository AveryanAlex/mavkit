/// Generates a PyO3 subscription class for an `ObservationSubscription<$rust_ty>`.
///
/// The generated struct exposes `recv()`, `__aiter__()`, and `__anext__()` so it can
/// be iterated with `async for` in Python.  Every call to `recv` (or `__anext__`) locks
/// the inner subscription, awaits the next value, converts it with `<$py_ty>::from`,
/// and raises `StopAsyncIteration` when the sender side is dropped.
macro_rules! py_subscription {
    ($struct_name:ident, $rust_ty:ty, $py_ty:ty, $py_name:literal, $closed_msg:literal) => {
        #[pyo3::pyclass(name = $py_name, frozen, skip_from_py_object)]
        pub struct $struct_name {
            pub(crate) inner:
                std::sync::Arc<tokio::sync::Mutex<mavkit::ObservationSubscription<$rust_ty>>>,
        }

        #[pyo3::pymethods]
        impl $struct_name {
            fn recv<'py>(
                &self,
                py: pyo3::Python<'py>,
            ) -> pyo3::PyResult<pyo3::Bound<'py, pyo3::types::PyAny>> {
                let inner = self.inner.clone();
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let mut guard = inner.lock().await;
                    match guard.recv().await {
                        Some(value) => Ok(<$py_ty>::from(value)),
                        None => Err(pyo3::exceptions::PyStopAsyncIteration::new_err($closed_msg)),
                    }
                })
            }

            fn __aiter__(slf: pyo3::PyRef<'_, Self>) -> pyo3::PyRef<'_, Self> {
                slf
            }

            fn __anext__<'py>(
                slf: pyo3::PyRef<'py, Self>,
                py: pyo3::Python<'py>,
            ) -> pyo3::PyResult<pyo3::Bound<'py, pyo3::types::PyAny>> {
                slf.recv(py)
            }
        }
    };
}

/// Generates a matched subscription + observation-handle pair for an `ObservationHandle<$rust_ty>`.
///
/// Both the subscription and the handle are generated from a single invocation.  Use this macro
/// when the Python type converts 1:1 from the Rust type via `From`.
///
/// For handles that cannot use this macro, write the subscription and handle manually:
/// - `PyLinkStateHandle` — `subscribe()` needs extra `last_error_message` storage.
/// - `PyModeCatalogHandle` — `latest()`/`wait*()`/`recv` map a `Vec` element by element.
macro_rules! define_observation_wrapper {
    (
        $handle_name:ident,
        $subscription_name:ident,
        $rust_ty:ty,
        $py_ty:ty,
        $py_name:literal,
        $subscription_py_name:literal,
        $closed_message:literal
    ) => {
        // Subscription — inlined from py_subscription! to avoid a nested macro call that would
        // require py_subscription to be in scope at every call site.
        #[pyo3::pyclass(name = $subscription_py_name, frozen, skip_from_py_object)]
        pub struct $subscription_name {
            pub(crate) inner:
                std::sync::Arc<tokio::sync::Mutex<mavkit::ObservationSubscription<$rust_ty>>>,
        }

        #[pyo3::pymethods]
        impl $subscription_name {
            fn recv<'py>(
                &self,
                py: pyo3::Python<'py>,
            ) -> pyo3::PyResult<pyo3::Bound<'py, pyo3::types::PyAny>> {
                let inner = self.inner.clone();
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let mut guard = inner.lock().await;
                    match guard.recv().await {
                        Some(value) => Ok(<$py_ty>::from(value)),
                        None => Err(pyo3::exceptions::PyStopAsyncIteration::new_err(
                            $closed_message,
                        )),
                    }
                })
            }

            fn __aiter__(slf: pyo3::PyRef<'_, Self>) -> pyo3::PyRef<'_, Self> {
                slf
            }

            fn __anext__<'py>(
                slf: pyo3::PyRef<'py, Self>,
                py: pyo3::Python<'py>,
            ) -> pyo3::PyResult<pyo3::Bound<'py, pyo3::types::PyAny>> {
                slf.recv(py)
            }
        }

        // Handle
        #[pyo3::pyclass(name = $py_name, frozen, skip_from_py_object)]
        #[derive(Clone)]
        pub struct $handle_name {
            inner: mavkit::ObservationHandle<$rust_ty>,
        }

        impl $handle_name {
            pub(crate) fn new(inner: mavkit::ObservationHandle<$rust_ty>) -> Self {
                Self { inner }
            }
        }

        #[pyo3::pymethods]
        impl $handle_name {
            fn latest(&self) -> Option<$py_ty> {
                self.inner.latest().map(Into::into)
            }

            fn wait<'py>(
                &self,
                py: pyo3::Python<'py>,
            ) -> pyo3::PyResult<pyo3::Bound<'py, pyo3::types::PyAny>> {
                let inner = self.inner.clone();
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let value = inner.wait().await.map_err(crate::error::to_py_err)?;
                    Ok(<$py_ty>::from(value))
                })
            }

            fn wait_timeout<'py>(
                &self,
                py: pyo3::Python<'py>,
                timeout_secs: f64,
            ) -> pyo3::PyResult<pyo3::Bound<'py, pyo3::types::PyAny>> {
                let inner = self.inner.clone();
                let timeout = crate::error::duration_from_secs(timeout_secs)?;
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let value = inner
                        .wait_timeout(timeout)
                        .await
                        .map_err(crate::error::to_py_err)?;
                    Ok(<$py_ty>::from(value))
                })
            }

            fn subscribe(&self) -> $subscription_name {
                $subscription_name {
                    inner: std::sync::Arc::new(tokio::sync::Mutex::new(self.inner.subscribe())),
                }
            }
        }
    };
}

/// Generates a `mavkit::Vehicle` plan-handle wrapper used by mission/fence/rally bindings.
///
/// This is intentionally specialized, not a general PyO3 wrapper macro. It assumes:
/// - the wrapper owns a `mavkit::Vehicle`,
/// - state wrappers implement `From<...>`,
/// - plan wrappers expose an `inner` field,
/// - operation wrappers store `inner: Arc<_>`, and
/// - `crate::vehicle::vehicle_label` is the desired `__repr__` label source.
///
/// The generated handle includes common observation and plan transfer methods, while allowing
/// each handle to inject additional `#[pymethods]` items (e.g. mission verify/set_current,
/// fence/rally support accessors). Keep the `extras` block small so the macro stays readable.
macro_rules! define_vehicle_plan_handle {
    (
        handle = $handle_name:ident,
        py_name = $py_name:literal,
        state = $state_py_ty:ty,
        subscription = $subscription_ty:ident,
        plan = $plan_ty:ty,
        upload_op = $upload_op_ty:ident,
        download_op = $download_op_ty:ident,
        clear_op = $clear_op_ty:ident,
        access = $accessor:ident,
        extras = { $($extra_methods:item)* }
    ) => {
        #[pyo3::pyclass(name = $py_name, frozen, skip_from_py_object)]
        #[derive(Clone)]
        pub struct $handle_name {
            inner: mavkit::Vehicle,
        }

        impl $handle_name {
            pub(crate) fn new(inner: mavkit::Vehicle) -> Self {
                Self { inner }
            }
        }

        #[pyo3::pymethods]
        impl $handle_name {
            fn latest(&self) -> Option<$state_py_ty> {
                self.inner.$accessor().latest().map(Into::into)
            }

            fn wait<'py>(
                &self,
                py: pyo3::Python<'py>,
            ) -> pyo3::PyResult<pyo3::Bound<'py, pyo3::types::PyAny>> {
                let vehicle = self.inner.clone();
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let inner = vehicle.$accessor().wait().await;
                    Ok(<$state_py_ty>::from(inner))
                })
            }

            fn wait_timeout<'py>(
                &self,
                py: pyo3::Python<'py>,
                timeout_secs: f64,
            ) -> pyo3::PyResult<pyo3::Bound<'py, pyo3::types::PyAny>> {
                let vehicle = self.inner.clone();
                let timeout = crate::error::duration_from_secs(timeout_secs)?;
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let inner = vehicle
                        .$accessor()
                        .wait_timeout(timeout)
                        .await
                        .map_err(crate::error::to_py_err)?;
                    Ok(<$state_py_ty>::from(inner))
                })
            }

            fn subscribe(&self) -> $subscription_ty {
                $subscription_ty {
                    inner: std::sync::Arc::new(tokio::sync::Mutex::new(
                        self.inner.$accessor().subscribe(),
                    )),
                }
            }

            fn upload(&self, plan: &$plan_ty) -> pyo3::PyResult<$upload_op_ty> {
                let vehicle = self.inner.clone();
                let plan = plan.inner.clone();
                let op = pyo3_async_runtimes::tokio::get_runtime()
                    .block_on(async move { vehicle.$accessor().upload(plan) })
                    .map_err(crate::error::to_py_err)?;
                Ok($upload_op_ty {
                    inner: std::sync::Arc::new(op),
                })
            }

            fn download(&self) -> pyo3::PyResult<$download_op_ty> {
                let vehicle = self.inner.clone();
                let op = pyo3_async_runtimes::tokio::get_runtime()
                    .block_on(async move { vehicle.$accessor().download() })
                    .map_err(crate::error::to_py_err)?;
                Ok($download_op_ty {
                    inner: std::sync::Arc::new(op),
                })
            }

            fn clear(&self) -> pyo3::PyResult<$clear_op_ty> {
                let vehicle = self.inner.clone();
                let op = pyo3_async_runtimes::tokio::get_runtime()
                    .block_on(async move { vehicle.$accessor().clear() })
                    .map_err(crate::error::to_py_err)?;
                Ok($clear_op_ty {
                    inner: std::sync::Arc::new(op),
                })
            }

            $($extra_methods)*

            fn __repr__(&self) -> String {
                format!(concat!($py_name, "({})"), crate::vehicle::vehicle_label(&self.inner))
            }
        }
    };
}

/// Generates a frozen pyclass wrapper with keyword-only constructor, getters, and `__repr__`.
///
/// Covers the common case where every field is a scalar passed straight through to the
/// inner Rust struct. Wrappers with custom transforms (enum fields, GeoPoint3d, defaults,
/// fallible constructors) should remain hand-written.
macro_rules! py_frozen_wrapper {
    (
        $py_struct:ident wraps $($inner_seg:ident)::+ as $py_name:literal {
            $($field:ident : $field_ty:ty),+ $(,)?
        }
    ) => {
        #[pyo3::pyclass(name = $py_name, frozen, from_py_object)]
        #[derive(Clone)]
        pub struct $py_struct {
            pub(crate) inner: $($inner_seg)::+,
        }

        #[pyo3::pymethods]
        impl $py_struct {
            #[new]
            #[pyo3(signature = (*, $($field),+))]
            fn new($($field: $field_ty),+) -> Self {
                Self {
                    inner: $($inner_seg)::+ { $($field),+ },
                }
            }

            $(
                #[getter]
                fn $field(&self) -> $field_ty {
                    self.inner.$field
                }
            )+

            fn __repr__(&self) -> String {
                format!(
                    concat!($py_name, "({})"),
                    [$(format!(concat!(stringify!($field), "={:?}"), self.inner.$field)),+].join(", ")
                )
            }
        }
    };
}

/// Runs a Rust future in Python and maps `Result<(), VehicleError>` to `None`.
///
/// Use this for thin PyO3 async wrappers that only clone/capture values and then
/// forward to an async Rust API returning `Result<(), mavkit::VehicleError>`.
///
/// Example:
/// `py_async_unit!(py, vehicle = self.inner.clone(); vehicle.ardupilot().reboot())`
macro_rules! py_async_unit {
    ($py:expr, $($name:ident = $value:expr),+ ; $future:expr $(,)?) => {{
        $(let $name = $value;)+
        ::pyo3_async_runtimes::tokio::future_into_py($py, async move {
            let (): () = $future.await.map_err($crate::error::to_py_err)?;
            Ok(())
        })
    }};
}

pub(crate) use define_observation_wrapper;
pub(crate) use define_vehicle_plan_handle;
pub(crate) use py_async_unit;
pub(crate) use py_frozen_wrapper;
pub(crate) use py_subscription;
