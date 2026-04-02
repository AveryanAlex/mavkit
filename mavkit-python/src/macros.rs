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

pub(crate) use define_observation_wrapper;
pub(crate) use py_subscription;
