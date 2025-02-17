#
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#

from dataclasses import dataclass
from typing import Optional


@dataclass()
class Difference:
    added_nodes: Optional[dict]
    removed_nodes: Optional[dict]
    common_nodes: Optional[dict]
